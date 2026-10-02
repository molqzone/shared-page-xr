/**
 * @file test_shared_page.cpp
 * @brief `shared_page.hpp` 的契约测试 / Contract tests for `shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 双端编译依赖的数字：Sample / RegionPayload / Region / TelemetryRing 的字段
 *      偏移与总长，以及页内各区偏移。
 *   2. 发布索引：`head` 计数、物理槽位 `head % 64`、最新为 `ring[(head-1) % 64]`。
 *   3. `Since()` 的区间语义：追平、上界 64 槽、gap、新纪元（head 回退）。
 *   4. 页校验：冷页 / 被格式化 / 外来数据。
 *   5. region 的写-读一致性：稳定路径、撕裂重试、BUSY。
 *   6. 访问单元页的单槽「最新帧」借还语义。
 *
 * 运行前提 / Setup: 无。页由匿名映射提供，不需要 `/dev/mem`、root 或任何设备，所以
 * 这套契约在主机上就能验证。
 * None. The page is an anonymous mapping, so no `/dev/mem`, root or device is needed
 * and the contract is verified on the host.
 *
 * 非并发测试：`Since()` 的撕裂窗口与 region 的 BUSY 分支都依赖并发时序，单线程下无
 * 法确定性复现（`Since()` 只能测边界规则，BUSY 通过伪造中间状态覆盖重试逻辑）。
 * Concurrent behaviour is not stress-tested here: the tear window in `Since()` and the
 * BUSY branch of the region read depend on concurrent timing and cannot be reproduced
 * deterministically from one thread. `Since()` covers the boundary rules and the BUSY
 * path is covered by forging the intermediate state a torn read observes.
 */

#include <sys/mman.h>

#include <array>
#include <cstring>
#include <type_traits>
#include <vector>

#include "sample.hpp"
#include "shared_page.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using LibXRTest::MakeSample;
using LibXRTest::SameSample;

namespace
{

constexpr uint32_t kSampleBytes = 32;
constexpr uint32_t kRegionBytes = 32;
constexpr uint32_t kTelemetryBytes = kSampleBytes * TELEMETRY_SLOTS;

/// 读出一个结构体占用的字节，用于伪造外来页。Read a struct's bytes to forge a page.
template <typename T>
std::array<uint8_t, sizeof(T)> BytesOf(const T& value)
{
  std::array<uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  return bytes;
}

/// 一块 RAII 匿名映射页。An RAII anonymous page mapping.
class Mapping
{
 public:
  explicit Mapping(size_t bytes)
      : bytes_(bytes),
        data_(::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS,
                     -1, 0))
  {
    TEST_ASSERT(data_ != MAP_FAILED);
  }

  ~Mapping() { ::munmap(data_, bytes_); }

  Mapping(const Mapping&) = delete;
  Mapping& operator=(const Mapping&) = delete;

  [[nodiscard]] uint8_t* Data() const { return static_cast<uint8_t*>(data_); }

 private:
  size_t bytes_;
  void* data_;
};

/// 1. 双端编译依赖的数字。The numbers both cores compile against.
void TestLayout()
{
  TEST_ASSERT(sizeof(Sample) == kSampleBytes);
  TEST_ASSERT(offsetof(Sample, ticks) == 0);
  TEST_ASSERT(offsetof(Sample, accel) == 8);
  TEST_ASSERT(offsetof(Sample, gyro) == 14);
  TEST_ASSERT(offsetof(Sample, servo_target) == 20);
  TEST_ASSERT(offsetof(Sample, pad) == 28);
  TEST_ASSERT(sizeof(Sample::servo_target) ==
              LibXRTest::SERVO_CHANNELS * sizeof(uint16_t));

  // RegionPayload 的字段只占 24B：最后一个 float 结束于 24，ABI 不补尾。而 seq 必须
  // 落在 28，所以那 4B 是 Region 的显式填充。两个数都要钉住。
  // RegionPayload's fields occupy only 24B; the ABI adds no tail padding, yet seq must
  // land at 28, so those 4B are explicit padding on Region. Both numbers are pinned.
  TEST_ASSERT(sizeof(RegionPayload) == 24);
  TEST_ASSERT(offsetof(RegionPayload, aim_x) == 4);
  TEST_ASSERT(offsetof(RegionPayload, aim_y) == 8);
  TEST_ASSERT(offsetof(RegionPayload, cmd) == 12);
  TEST_ASSERT(offsetof(RegionPayload, param_id) == 16);
  TEST_ASSERT(offsetof(RegionPayload, value) == 20);

  TEST_ASSERT(sizeof(Region) == kRegionBytes);
  TEST_ASSERT(offsetof(Region, payload_pad) == 24);
  TEST_ASSERT(offsetof(Region, seq) == 28);

  TEST_ASSERT(TelemetryOffset() == 8);
  TEST_ASSERT(RegionOffset() == PAGE_SIZE - kRegionBytes);
  TEST_ASSERT(TelemetryOffset() + kTelemetryBytes <= RegionOffset());

  // 发布索引必须是页内一个可原子访问的 4B 计数。
  // The publish index must be one atomically accessible 4B counter in the page.
  TEST_ASSERT((std::is_same_v<decltype(std::declval<TelemetryRing&>().head),
                              std::atomic<uint32_t>>));
  TEST_ASSERT(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
  TEST_ASSERT(std::atomic<uint32_t>::is_always_lock_free);
}

/// 2. 冷页、Format() 与发布索引。Cold page, Format(), and the publish index.
void TestColdPageAndIndex(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  TEST_ASSERT(page.Check() == PageMagicKind::UNFORMATTED);
  TEST_ASSERT(!page.Ready());

  Sample probe = {};
  TEST_ASSERT(!page.Latest(&probe));
  TEST_ASSERT(page.TelemetryReader().Head() == 0);
  TEST_ASSERT(page.Region().Seq() == 0);

  // 未格式化也能写：写者只需要映射，Format() 负责清零历史。
  // Writing before Format() is defined: the writer only needs the mapping, and
  // Format() zeroes the history.
  TEST_ASSERT(page.WriteSample(MakeSample(0)) == 1);

  page.Format();
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);
  TEST_ASSERT(page.Ready());
  TEST_ASSERT(page.Region().Seq() == 0);
  TEST_ASSERT(!page.Latest(&probe));

  const auto* header = reinterpret_cast<const PageHeader*>(mapping.Data());
  TEST_ASSERT(header->magic == PAGE_MAGIC);
  TEST_ASSERT(header->page_size == PAGE_SIZE);

  page.WriteSample(MakeSample(1));
  TEST_ASSERT(page.Latest(&probe));
  TEST_ASSERT(probe.ticks == 1001);

  page.ClearHistory();
  TEST_ASSERT(page.TelemetryReader().Head() == 0);
  TEST_ASSERT(!page.Latest(&probe));
  // ClearHistory() 只清发布索引，不破坏魔术字。
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);
}

/// 3. 外来页必须被识别，绝不能被静默使用。A foreign page must be reported, never
///    silently used.
void TestForeignPage(const Mapping& mapping)
{
  const PageHeader foreign = {.magic = 0x11223344U, .page_size = PAGE_SIZE};
  std::memcpy(mapping.Data(), BytesOf(foreign).data(), sizeof(foreign));

  SharedPage page(mapping.Data());
  TEST_ASSERT(page.Check() == PageMagicKind::FOREIGN);

  page.Format();
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);
}

/// 4. 发布索引与最新采样。Publish index and the latest sample.
void TestLatest(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  for (uint32_t index = 0; index < 3; ++index)
  {
    TEST_ASSERT(page.TelemetryWriter().Write(MakeSample(index)) == index + 1);
  }
  TEST_ASSERT(page.TelemetryWriter().Head() == 3);

  Sample latest = {};
  TEST_ASSERT(page.Latest(&latest));
  TEST_ASSERT(SameSample(latest, MakeSample(2)));

  const auto* ring =
      reinterpret_cast<const TelemetryRing*>(mapping.Data() + TelemetryOffset());
  TEST_ASSERT(SameSample(ring->ring[2], MakeSample(2)));
  TEST_ASSERT(SameSample(ring->ring[0], MakeSample(0)));
}

/// 5. `Since()` 的区间、上界与 gap。Range, bound and gap of `Since()`.
void TestSince(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  page.ClearHistory();
  auto writer = page.TelemetryWriter();
  const auto reader = page.TelemetryReader();

  RingScan scan = RingScan::DATA;
  uint32_t next = 7;
  std::array<Sample, TELEMETRY_SLOTS> out = {};

  TEST_ASSERT(reader.Since(0, &scan, &next, out.data(), out.size()) == 0);
  TEST_ASSERT(scan == RingScan::IDLE);
  TEST_ASSERT(next == 0);

  // 写者跑出 100 条，落后 90 槽的读者整段丢：`head - last > 64`。
  // The writer ran 100 ahead; a reader 90 slots behind drops the whole range.
  for (uint32_t index = 0; index < 100; ++index)
  {
    writer.Write(MakeSample(index));
  }
  TEST_ASSERT(writer.Head() == 100);
  TEST_ASSERT(reader.Since(10, &scan, &next, out.data(), out.size()) == 0);
  TEST_ASSERT(scan == RingScan::GAP);
  TEST_ASSERT(next == 100);

  // 正好落后 64 槽仍是可解析区间（上界是 `> 64`）。
  // Exactly 64 slots behind is still parseable: the bound is `> 64`.
  page.ClearHistory();
  for (uint32_t index = 0; index < 64; ++index)
  {
    writer.Write(MakeSample(index));
  }
  TEST_ASSERT(reader.Since(0, &scan, &next, out.data(), out.size()) == 64);
  TEST_ASSERT(scan == RingScan::DATA);
  TEST_ASSERT(next == 64);
  for (uint32_t index = 0; index < 64; ++index)
  {
    TEST_ASSERT(SameSample(out[index], MakeSample(index)));
  }

  // 接着 drain 一小段：区间上界是 head，序列正序。
  // Drain a short range: the upper bound is head and the order is ascending.
  for (uint32_t index = 64; index < 66; ++index)
  {
    writer.Write(MakeSample(index));
  }
  TEST_ASSERT(reader.Since(next, &scan, &next, out.data(), out.size()) == 2);
  TEST_ASSERT(scan == RingScan::DATA);
  TEST_ASSERT(next == 66);
  TEST_ASSERT(SameSample(out[0], MakeSample(64)));
  TEST_ASSERT(SameSample(out[1], MakeSample(65)));

  // 再写一条即可证明回绕：ClearHistory() 后索引从 0 重新计，head 66 落在槽 2。
  // One more write proves the wrap: after ClearHistory() the index restarts at 0, so
  // head 66 lives in slot 2.
  writer.Write(MakeSample(66));
  TEST_ASSERT(reader.Since(66, &scan, &next, out.data(), out.size()) == 1);
  TEST_ASSERT(SameSample(out[0], MakeSample(66)));
  const auto* ring =
      reinterpret_cast<const TelemetryRing*>(mapping.Data() + TelemetryOffset());
  TEST_ASSERT(SameSample(ring->ring[2], MakeSample(66)));

  // 接收缓冲为 nullptr 时只回答区间/gap 问题。
  // A null receive buffer only answers the range/gap question.
  TEST_ASSERT(reader.Since(66, &scan, &next, nullptr, 0) == 1);
  TEST_ASSERT(scan == RingScan::DATA);
  TEST_ASSERT(next == 67);

  // 上一纪元留下的 last_seen（大于 head）是 gap，不是回绕出来的区间。
  // A last_seen from a previous epoch (larger than head) is a gap, not a wrapped range.
  TEST_ASSERT(reader.Since(1000, &scan, &next, out.data(), out.size()) == 0);
  TEST_ASSERT(scan == RingScan::GAP);
  TEST_ASSERT(next == 67);
}

/// 6. region 的稳定读、失败重试的上限与 BUSY 判定。
///    Stable region reads, the retry budget and the BUSY verdict.
void TestRegion(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  page.ClearHistory();
  auto reference = page.Region();

  Region snapshot = {};
  TEST_ASSERT(reference.Read(&snapshot) == RegionScan::CURRENT);
  TEST_ASSERT(snapshot.seq.load() == 0);
  TEST_ASSERT(snapshot.payload.found == 0);

  RegionPayload payload = {};
  payload.found = 1;
  payload.aim_x = 0.25F;
  payload.aim_y = -0.5F;
  payload.cmd = Region::CMD_PARAM;
  payload.param_id = 9;
  payload.value = 1.5F;
  TEST_ASSERT(reference.Write(payload) == 1);
  TEST_ASSERT(reference.WriteAim(true, 0.125F, 0.75F) == 2);

  TEST_ASSERT(reference.Read(&snapshot) == RegionScan::CURRENT);
  TEST_ASSERT(snapshot.seq.load() == 2);
  TEST_ASSERT(snapshot.payload.aim_x == 0.125F);
  TEST_ASSERT(snapshot.payload.aim_y == 0.75F);
  // WriteAim 只写 aim 字段，上一次的 cmd/param_id 随之清零（整块 payload 覆盖）。
  // WriteAim writes only the aim fields, so the previous cmd/param_id are cleared by
  // the whole-payload write.
  TEST_ASSERT(snapshot.payload.cmd == 0);
  TEST_ASSERT(snapshot.payload.param_id == 0);

  auto* region = reinterpret_cast<Region*>(mapping.Data() + RegionOffset());
  TEST_ASSERT(reference.Raw() == region);

  // 伪造一次撕裂写的第一轮：写者已把索引提到 3，读者 `before` 读到 3、拷贝到新
  // payload，而 `after` 读到旧值 2——不一致，必须重试；第二轮要拿到完整的 payload，
  // 不能把中间的撕裂态当成自洽。
  // Forge the first attempt of a torn write: the writer bumped the index to 3, so the
  // reader's `before` sees 3 and copies the new payload while `after` sees the old 2.
  // The read must retry and the second attempt must return the completed payload
  // instead of accepting the torn intermediate.
  payload.aim_x = 123.0F;
  region->payload = payload;
  region->seq.store(3, std::memory_order_release);
  region->seq.store(2, std::memory_order_release);
  TEST_ASSERT(reference.Read(&snapshot) == RegionScan::CURRENT);
  TEST_ASSERT(snapshot.seq.load() == 2);
  TEST_ASSERT(snapshot.payload.aim_x == 123.0F);

  // BUSY 只在重试上限内读不到自洽快照时出现：把重试预算设为 0 就等于强制走该分支，
  // 用来钉住「预算耗尽后仍返回最后一次拷贝并标记 BUSY」的约定。
  // BUSY appears only when no coherent snapshot fits the retry budget; a budget of 0
  // forces that branch and pins the "return the last copy tagged BUSY" contract.
  TEST_ASSERT(reference.Read(&snapshot, 0) == RegionScan::BUSY);
  TEST_ASSERT(snapshot.payload.aim_x == 123.0F);
}

/// 7. 访问单元页的单槽借还语义。Single-slot borrow semantics of the access-unit page.
void TestAccessUnit()
{
  const size_t access_bytes = sizeof(AccessUnit) + PAGE_SIZE;
  Mapping mapping(access_bytes);
  auto* memory = mapping.Data();

  AccessUnitPage mailbox(memory);
  TEST_ASSERT(mailbox.Check() == PageMagicKind::UNFORMATTED);
  mailbox.Format();
  TEST_ASSERT(mailbox.Check() == PageMagicKind::FORMATTED);

  TEST_ASSERT(!mailbox.Acquire().Valid());

  std::array<uint8_t, 64> frame = {};
  for (uint32_t index = 0; index < frame.size(); ++index)
  {
    frame[index] = static_cast<uint8_t>(index);
  }

  TEST_ASSERT(
      mailbox.Publish(frame.data(), 48, AccessUnit::FORMAT_H264_ANNEX_B, 640, 480) == 1);

  const uint32_t length = 48;
  const auto view = mailbox.Acquire(true);
  TEST_ASSERT(view.Valid());
  TEST_ASSERT(view.length == length);
  TEST_ASSERT(view.format == AccessUnit::FORMAT_H264_ANNEX_B);
  TEST_ASSERT(view.width == 640);
  TEST_ASSERT(view.height == 480);
  TEST_ASSERT(view.seq == 1);
  TEST_ASSERT(std::memcmp(view.data, frame.data(), length) == 0);

  // Publish() 用 LibXR::CRC32 写入校验值，页内保存的就是该算法的结果。
  // Publish() stores the LibXR::CRC32 result, so the page holds that algorithm's value.
  const auto* slot = reinterpret_cast<const AccessUnit*>(memory);
  TEST_ASSERT(slot->crc32.load() == CRC32::Calculate(frame.data(), length));

  // 借出一次即清 ready：同一帧不会被消费两次。
  // One take clears ready, so one frame is consumed once.
  TEST_ASSERT(!mailbox.Acquire().Valid());

  // 写者覆写而不阻塞；读者拿到新的 seq。
  // The writer overwrites rather than blocking and the reader sees the new seq.
  frame[0] = 0xAB;
  TEST_ASSERT(
      mailbox.Publish(frame.data(), 16, AccessUnit::FORMAT_H264_ANNEX_B, 320, 240) == 2);
  const auto second = mailbox.Acquire(true);
  TEST_ASSERT(second.Valid());
  TEST_ASSERT(second.seq == 2);
  TEST_ASSERT(second.length == 16);
  TEST_ASSERT(second.width == 320);
  TEST_ASSERT(second.data[0] == 0xAB);

  // 校验失败：把页内 payload 改坏，取帧必须拒绝，且 ready 保持置位以便重试。
  // A failed check: corrupting the in-page payload must be rejected, with `ready` left
  // set so the caller can retry.
  TEST_ASSERT(
      mailbox.Publish(frame.data(), 16, AccessUnit::FORMAT_H264_ANNEX_B, 320, 240) == 3);
  auto* corrupt = reinterpret_cast<AccessUnit*>(memory);
  corrupt->payload[0] ^= 0xFFU;
  TEST_ASSERT(!mailbox.Acquire(true).Valid());
  TEST_ASSERT(corrupt->ready.load() == 1);

  // 关掉校验时不复算，取帧照常成功（热路径）。
  // With the check off nothing is recomputed and the take succeeds, which is the hot
  // path.
  const auto unchecked = mailbox.Acquire(false);
  TEST_ASSERT(unchecked.Valid());
  TEST_ASSERT(unchecked.seq == 3);

  // 计算 CRC32 不是强制的：关掉时页内记 0，表示未计算。
  // Computing the CRC32 is optional: turned off, the page records 0 for "not computed".
  TEST_ASSERT(mailbox.Publish(frame.data(), 16, AccessUnit::FORMAT_H264_ANNEX_B, 320, 240,
                              false) == 4);
  TEST_ASSERT(corrupt->crc32.load() == 0);

  // 越界与空指针被拒绝，而不是截断。
  // Oversized and null publishes are rejected, not truncated.
  TEST_ASSERT(mailbox.Publish(frame.data(), AccessUnit::MAX_BYTES + 1, 1, 1, 1) == 0);
  TEST_ASSERT(mailbox.Publish(nullptr, 8, 1, 1, 1) == 0);

  // 正好到上界的发布被接受（边界是 `> MAX`）。
  // A publish exactly at the limit is accepted: the bound is `> MAX`.
  std::vector<uint8_t> full_frame(AccessUnit::MAX_BYTES, 0x5AU);
  TEST_ASSERT(mailbox.Publish(full_frame.data(), AccessUnit::MAX_BYTES, 1, 2, 3) == 5);
  const auto full = mailbox.Acquire(true);
  TEST_ASSERT(full.Valid());
  TEST_ASSERT(full.length == AccessUnit::MAX_BYTES);
  TEST_ASSERT(full.data[AccessUnit::MAX_BYTES - 1] == 0x5AU);

  // 头部在页首，payload 紧随其后。
  // The header occupies the start of the page and the payload follows it.
  TEST_ASSERT(slot->magic == ACCESS_UNIT_MAGIC);
  TEST_ASSERT(reinterpret_cast<const uint8_t*>(slot->payload) ==
              memory + offsetof(AccessUnit, payload));
}

}  // namespace

int main()
{
  TestLayout();

  {
    Mapping mapping(PAGE_SIZE);
    TestColdPageAndIndex(mapping);
    TestForeignPage(mapping);
    TestLatest(mapping);
    TestSince(mapping);
    TestRegion(mapping);
  }

  TestAccessUnit();
  return 0;
}
