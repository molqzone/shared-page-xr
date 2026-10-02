/**
 * @file test_shared_page.cpp
 * @brief `shared_page.hpp` 的契约测试 / Contract tests for `shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 双端编译依赖的数字：Sample / Region / TelemetryRing 的字段
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

constexpr uint32_t SAMPLE_BYTES = 40;
constexpr uint32_t REGION_BYTES = 32;
constexpr uint32_t TELEMETRY_BYTES = SAMPLE_BYTES * TELEMETRY_SLOTS;

struct TestProtocolPayload
{
  uint8_t found = 0;
  uint8_t pad[3] = {};
  float aim_x = 0;
  float aim_y = 0;
  uint8_t cmd = 0;
  uint8_t pad2[3] = {};
  uint16_t param_id = 0;
  float value = 0;
};

static_assert(sizeof(TestProtocolPayload) == REGION_PAYLOAD_BYTES);

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
  TEST_ASSERT(sizeof(Sample) == SAMPLE_BYTES);
  TEST_ASSERT(offsetof(Sample, ticks) == 0);
  TEST_ASSERT(offsetof(Sample, accel) == 8);
  TEST_ASSERT(offsetof(Sample, gyro) == 14);
  TEST_ASSERT(offsetof(Sample, temperature) == 20);
  TEST_ASSERT(offsetof(Sample, servo_target) == 22);
  TEST_ASSERT(offsetof(Sample, servo_actual) == 30);
  TEST_ASSERT(offsetof(Sample, pad) == 38);
  TEST_ASSERT(sizeof(Sample::servo_target) ==
              LibXRTest::SERVO_CHANNELS * sizeof(uint16_t));
  TEST_ASSERT(sizeof(Sample::servo_actual) ==
              LibXRTest::SERVO_CHANNELS * sizeof(uint16_t));

  TEST_ASSERT(sizeof(Region) == REGION_BYTES);
  TEST_ASSERT(offsetof(Region, write_state) == 24);
  TEST_ASSERT(offsetof(Region, seq) == 28);

  TEST_ASSERT(TelemetryOffset() == 8);
  TEST_ASSERT(RegionOffset() == PAGE_SIZE - REGION_BYTES);
  TEST_ASSERT(TelemetryOffset() + TELEMETRY_BYTES <= RegionOffset());

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

  auto* writable_ring =
      reinterpret_cast<TelemetryRing*>(mapping.Data() + TelemetryOffset());
  writable_ring->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(page.TelemetryWriter().Write(MakeSample(3)) == 0);
  writable_ring->write_state.store(0, std::memory_order_release);
}

/// 5. `Since()` 的区间、上界与 gap。Range, bound and gap of `Since()`.
void TestSince(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  page.ClearHistory();
  auto writer = page.TelemetryWriter();
  const auto reader = page.TelemetryReader();

  SinceResult result = {};
  std::array<Sample, TELEMETRY_SLOTS> out = {};

  auto* telemetry_ring =
      reinterpret_cast<TelemetryRing*>(mapping.Data() + TelemetryOffset());
  telemetry_ring->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::BUSY);
  TEST_ASSERT(result.dropped == 0);
  telemetry_ring->write_state.store(0, std::memory_order_release);

  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 0);

  // 写者跑出 100 条，落后 90 槽的读者整段丢：`head - last > 64`。
  // The writer ran 100 ahead; a reader 90 slots behind drops the whole range.
  for (uint32_t index = 0; index < 100; ++index)
  {
    writer.Write(MakeSample(index));
  }
  TEST_ASSERT(writer.Head() == 100);
  TEST_ASSERT(reader.Since(10, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 90);
  TEST_ASSERT(result.next == 100);

  // 正好落后 64 槽仍是可解析区间（上界是 `> 64`）。
  // Exactly 64 slots behind is still parseable: the bound is `> 64`.
  page.ClearHistory();
  for (uint32_t index = 0; index < 64; ++index)
  {
    writer.Write(MakeSample(index));
  }
  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 64);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 64);
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
  TEST_ASSERT(reader.Since(64, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 2);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 66);
  TEST_ASSERT(SameSample(out[0], MakeSample(64)));
  TEST_ASSERT(SameSample(out[1], MakeSample(65)));

  // 再写一条即可证明回绕：ClearHistory() 后索引从 0 重新计，head 66 落在槽 2。
  // One more write proves the wrap: after ClearHistory() the index restarts at 0, so
  // head 66 lives in slot 2.
  writer.Write(MakeSample(66));
  TEST_ASSERT(reader.Since(66, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 1);
  TEST_ASSERT(result.next == 67);
  TEST_ASSERT(SameSample(out[0], MakeSample(66)));
  const auto* ring =
      reinterpret_cast<const TelemetryRing*>(mapping.Data() + TelemetryOffset());
  TEST_ASSERT(SameSample(ring->ring[2], MakeSample(66)));

  // 空缓冲仍回答区间问题，但没有样本可交付。
  // A null buffer still answers the range question, but delivers no samples.
  TEST_ASSERT(reader.Since(66, nullptr, TELEMETRY_SLOTS, &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 1);
  TEST_ASSERT(result.next == 67);

  // 容量小于区间时只交付容量内的一段，其余计入 dropped（调用方缓冲不足，不是历史覆写）。
  // 区间按「最新优先」拷贝，被裁掉的是最旧一端：tiny 里是最新的一条。
  // A capacity smaller than the range delivers only what fits and counts the rest as
  // dropped (caller capacity, not a history overwrite). The range is copied newest
  // first, so clamping drops the oldest end and tiny keeps the newest sample.
  std::array<Sample, 1> tiny = {};
  writer.Write(MakeSample(67));
  writer.Write(MakeSample(68));
  TEST_ASSERT(reader.Since(67, tiny.data(), tiny.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 1);
  TEST_ASSERT(result.dropped == 1);
  TEST_ASSERT(result.next == 69);
  TEST_ASSERT(SameSample(tiny[0], MakeSample(68)));

  // 上一纪元留下的 last_seen（大于 head）是 gap，不是回绕出来的区间。
  // A last_seen from a previous epoch (larger than head) is a gap, not a wrapped range.
  TEST_ASSERT(reader.Since(1000, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped > 0);
  TEST_ASSERT(result.next == 69);
}

/// 6. region 的稳定读、失败重试的上限与 BUSY 判定。
///    Stable region reads, the retry budget and the BUSY verdict.
void TestRegion(const Mapping& mapping)
{
  SharedPage page(mapping.Data());
  page.ClearHistory();
  auto reference = page.Region();

  Region snapshot = {};
  TEST_ASSERT(reference.Read(&snapshot) == ErrorCode::OK);
  TEST_ASSERT(snapshot.seq.load() == 0);
  TestProtocolPayload decoded = {};
  std::memcpy(&decoded, snapshot.payload, sizeof(decoded));
  TEST_ASSERT(decoded.found == 0);

  TestProtocolPayload payload = {};
  payload.found = 1;
  payload.aim_x = 0.25F;
  payload.aim_y = -0.5F;
  payload.cmd = 2;
  payload.param_id = 9;
  payload.value = 1.5F;
  TEST_ASSERT(reference.Write(payload) == 1);

  payload = {};
  payload.found = 1;
  payload.aim_x = 0.125F;
  payload.aim_y = 0.75F;
  TEST_ASSERT(reference.Write(payload) == 2);

  TEST_ASSERT(reference.Read(&snapshot) == ErrorCode::OK);
  TEST_ASSERT(snapshot.seq.load() == 2);
  std::memcpy(&decoded, snapshot.payload, sizeof(decoded));
  TEST_ASSERT(decoded.aim_x == 0.125F);
  TEST_ASSERT(decoded.aim_y == 0.75F);
  TEST_ASSERT(decoded.cmd == 0);
  TEST_ASSERT(decoded.param_id == 0);

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
  std::memcpy(region->payload, &payload, sizeof(payload));
  region->seq.store(3, std::memory_order_release);
  region->seq.store(2, std::memory_order_release);
  TEST_ASSERT(reference.Read(&snapshot) == ErrorCode::OK);
  TEST_ASSERT(snapshot.seq.load() == 2);
  std::memcpy(&decoded, snapshot.payload, sizeof(decoded));
  TEST_ASSERT(decoded.aim_x == 123.0F);

  // BUSY 只在重试上限内读不到自洽快照时出现：把重试预算设为 0 就等于强制走该分支，
  // 用来钉住「预算耗尽后仍返回最后一次拷贝并标记 BUSY」的约定。
  // BUSY appears only when no coherent snapshot fits the retry budget; a budget of 0
  // forces that branch and pins the "return the last copy tagged BUSY" contract.
  TEST_ASSERT(reference.Read(&snapshot, 0) == ErrorCode::BUSY);
  std::memcpy(&decoded, snapshot.payload, sizeof(decoded));
  TEST_ASSERT(decoded.aim_x == 123.0F);

  region->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(reference.Read(&snapshot) == ErrorCode::BUSY);
  std::memcpy(&decoded, snapshot.payload, sizeof(decoded));
  TEST_ASSERT(decoded.aim_x == 123.0F);
  TEST_ASSERT(reference.Write(payload) == 0);
  region->write_state.store(0, std::memory_order_release);
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

  auto* slot = reinterpret_cast<AccessUnit*>(memory);
  slot->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(mailbox.Publish(frame.data(), 1, 1, 1, 1) == 0);
  TEST_ASSERT(!mailbox.Acquire().Valid());
  slot->write_state.store(0, std::memory_order_release);

  TEST_ASSERT(
      mailbox.Publish(frame.data(), 48, AccessUnit::FORMAT_H264_ANNEX_B, 640, 480) == 1);

  const uint32_t length = 48;
  const auto view = mailbox.Acquire();
  TEST_ASSERT(view.Valid());
  TEST_ASSERT(view.length == length);
  TEST_ASSERT(view.format == AccessUnit::FORMAT_H264_ANNEX_B);
  TEST_ASSERT(view.width == 640);
  TEST_ASSERT(view.height == 480);
  TEST_ASSERT(view.seq == 1);
  TEST_ASSERT(std::memcmp(view.data, frame.data(), length) == 0);

  // 借出一次即清 ready：同一帧不会被消费两次。
  // One take clears ready, so one frame is consumed once.
  TEST_ASSERT(!mailbox.Acquire().Valid());

  // 写者覆写而不阻塞；读者拿到新的 seq。
  // The writer overwrites rather than blocking and the reader sees the new seq.
  frame[0] = 0xAB;
  TEST_ASSERT(
      mailbox.Publish(frame.data(), 16, AccessUnit::FORMAT_H264_ANNEX_B, 320, 240) == 2);
  const auto second = mailbox.Acquire();
  TEST_ASSERT(second.Valid());
  TEST_ASSERT(second.seq == 2);
  TEST_ASSERT(second.length == 16);
  TEST_ASSERT(second.width == 320);
  TEST_ASSERT(second.data[0] == 0xAB);

  // 契约不设 CRC：取帧即热路径，没有校验参数（一致性由 seq 双重读保证）。
  // The contract carries no CRC: the take is the hot path with no check parameter
  // (coherence comes from the double read of seq).
  TEST_ASSERT(
      mailbox.Publish(frame.data(), 16, AccessUnit::FORMAT_H264_ANNEX_B, 320, 240) == 3);
  const auto third = mailbox.Acquire();
  TEST_ASSERT(third.Valid());
  TEST_ASSERT(third.seq == 3);
  TEST_ASSERT(third.data[0] == 0xAB);

  // 越界与空指针被拒绝，而不是截断。
  // Oversized and null publishes are rejected, not truncated.
  TEST_ASSERT(mailbox.Publish(frame.data(), AccessUnit::MAX_BYTES + 1, 1, 1, 1) == 0);
  TEST_ASSERT(mailbox.Publish(nullptr, 8, 1, 1, 1) == 0);

  // 正好到上界的发布被接受（边界是 `> MAX`）。
  // A publish exactly at the limit is accepted: the bound is `> MAX`.
  std::vector<uint8_t> full_frame(AccessUnit::MAX_BYTES, 0x5AU);
  TEST_ASSERT(mailbox.Publish(full_frame.data(), AccessUnit::MAX_BYTES, 1, 2, 3) == 4);
  const auto full = mailbox.Acquire();
  TEST_ASSERT(full.Valid());
  TEST_ASSERT(full.length == AccessUnit::MAX_BYTES);
  TEST_ASSERT(full.data[AccessUnit::MAX_BYTES - 1] == 0x5AU);

  // 头部在页首，payload 紧随其后。
  // The header occupies the start of the page and the payload follows it.
  const auto* slot_view = reinterpret_cast<const AccessUnit*>(memory);
  TEST_ASSERT(slot_view->magic == ACCESS_UNIT_MAGIC);
  TEST_ASSERT(reinterpret_cast<const uint8_t*>(slot_view->payload) ==
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
