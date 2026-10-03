/**
 * @file test_shared_page.cpp
 * @brief `shared_page.hpp` 的契约测试 / Contract tests for `shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 双端编译依赖的数字：帧几何跟随注入的 format，`TelemetryRing` / `Region`
 *      跟随注入类型的字段偏移与总长、页内各区偏移、`PageLayout` 与页头同源。
 *   2. 发布索引：`head` 计数、物理槽位 `head % SLOT_COUNT`、最新为
 *      `ring[(head-1) % SLOT_COUNT]`。
 *   3. `Since()` 的区间语义：追平、上界 SLOT_COUNT 槽、gap、新纪元（head 回退）。
 *   4. 页校验：冷页 / 被格式化 / 外来数据 / 双端契约漂移（TAG、载荷尺寸、帧几何，
 *      均报 MISMATCH）。
 *   5. region 的写-读一致性：稳定路径、撕裂重试、BUSY。
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
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include "libxr_def.hpp"
#include "sample.hpp"
#include "shared_page.hpp"
#include "test_assert.hpp"

using namespace LibXR;
using LibXRTest::make_sample;
using LibXRTest::same_sample;
using Sample = LibXRTest::Sample;

namespace
{

/// 测试用高层 payload：24B 只是这个测试的选择，不是契约常量。
/// Test higher-layer payload: 24B is this test's choice, not a contract constant.
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

static_assert(sizeof(TestProtocolPayload) == 24);
static_assert(sizeof(TestProtocolPayload) % sizeof(uint32_t) == 0);

using TestFormat = LibXRTest::Format;
using TestPage = SharedPage<TestFormat, Sample, TestProtocolPayload>;

/// 另一份契约：同类型、不同 TAG，用来伪造双端 wire 版本漂移。
/// Another contract: same types, a different TAG, to forge wire version drift.
using TaggedPage = SharedPage<TestFormat, Sample, TestProtocolPayload, 0x54414731U>;

/// 再一份契约：payload 尺寸不同。
/// Yet another contract: a different payload size.
struct SmallPayload
{
  uint8_t bytes[16] = {};
};
using SmallPage = SharedPage<TestFormat, Sample, SmallPayload>;

/// 又一份契约：帧几何不同（文档稿的 4096B / 64 槽）——页头必须把这种漂移报出来。
/// Yet another contract: different frame geometry (the doc's 4096B/64 slots) -- the
/// header must report this drift too.
using WidePage = SharedPage<DocPageFormat, Sample, TestProtocolPayload>;

/// 读出一个结构体占用的字节，用于伪造外来页。Read a struct's bytes to forge a page.
template <typename T>
std::array<uint8_t, sizeof(T)> bytes_of(const T& value)
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
void test_layout()
{
  // wire frame 跟随注入的 format 与类型，而不是库里的常量。
  // The wire frames follow the injected format and types, not constants inside the
  // library.
  constexpr uint32_t slots = TestFormat::SLOT_COUNT;
  using Ring = TelemetryRing<Sample, slots>;
  TEST_ASSERT(TestFormat::PAGE_SIZE == 2048 && TestFormat::SLOT_COUNT == 32);
  TEST_ASSERT(sizeof(Ring) == sizeof(Sample) * slots + 8);
  TEST_ASSERT(offsetof(Ring, head) == sizeof(Sample) * slots);
  TEST_ASSERT(offsetof(Ring, write_state) == sizeof(Sample) * slots + 4);
  TEST_ASSERT(sizeof(Region<TestProtocolPayload>) == 32);
  TEST_ASSERT(offsetof(Region<TestProtocolPayload>, write_state) == 24);
  TEST_ASSERT(offsetof(Region<TestProtocolPayload>, seq) == 28);

  TEST_ASSERT(telemetry_offset() == 32);
  TEST_ASSERT((region_offset<TestFormat, TestProtocolPayload>() == 2048 - 32));
  TEST_ASSERT((telemetry_offset() + sizeof(Ring) <=
               region_offset<TestFormat, TestProtocolPayload>()));
  TEST_ASSERT((region_offset<TestFormat, SmallPayload>() ==
               TestFormat::PAGE_SIZE - sizeof(Region<SmallPayload>)));

  // 布局描述与页头同源：页上记录的几何就是这些数字。
  // The layout descriptor shares its numbers with the header: the geometry recorded
  // on the page is exactly these numbers.
  const PageLayout layout = TestPage::Layout();
  TEST_ASSERT(layout.abi_version == TestFormat::ABI_VERSION);
  TEST_ASSERT(layout.page_size == TestFormat::PAGE_SIZE);
  TEST_ASSERT(layout.slot_count == TestFormat::SLOT_COUNT);
  TEST_ASSERT(layout.sample_size == sizeof(Sample));
  TEST_ASSERT(layout.payload_size == sizeof(TestProtocolPayload));
  TEST_ASSERT((layout.region_offset ==
               region_offset<TestFormat, TestProtocolPayload>()));
  TEST_ASSERT(layout.tag == 0);
  TEST_ASSERT(TaggedPage::Layout().tag != layout.tag);
  TEST_ASSERT(SmallPage::Layout().payload_size != layout.payload_size);
  TEST_ASSERT(WidePage::Layout().page_size != layout.page_size);
  TEST_ASSERT(WidePage::Layout().slot_count != layout.slot_count);

  // 发布索引必须是页内一个可原子访问的 4B 计数。
  // The publish index must be one atomically accessible 4B counter in the page.
  TEST_ASSERT((std::is_same_v<decltype(std::declval<Ring&>().head),
                              std::atomic<uint32_t>>));
  TEST_ASSERT(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
  TEST_ASSERT(std::atomic<uint32_t>::is_always_lock_free);
}

void test_unbound_page()
{
  TestPage page;
  Sample sample = {};
  SinceResult result = {.written = 1, .dropped = 1, .next = 1};
  TestProtocolPayload payload = {};
  const auto reader = page.TelemetryReader();
  auto reference = page.Region();

  TEST_ASSERT(!page.Valid());
  TEST_ASSERT(page.WriteSample(sample) == 0);
  TEST_ASSERT(!page.Latest(&sample));
  TEST_ASSERT(reader.Since(0, &sample, 1, &result) == ErrorCode::PTR_NULL);
  TEST_ASSERT(result.written == 0 && result.dropped == 0 && result.next == 0);
  TEST_ASSERT(reader.Since(0, &sample, 1, nullptr) == ErrorCode::PTR_NULL);
  TEST_ASSERT(reference.Read(&payload, nullptr) == ErrorCode::PTR_NULL);
  TEST_ASSERT(reference.Read(static_cast<TestProtocolPayload*>(nullptr), nullptr) ==
              ErrorCode::PTR_NULL);
  TEST_ASSERT(reference.Write(payload) == 0);
  TEST_ASSERT(reference.Raw() == nullptr);
}

/// 2. 冷页、Format() 与发布索引。Cold page, Format(), and the publish index.
void test_cold_page_and_index(const Mapping& mapping)
{
  TestPage page(mapping.Data());
  TEST_ASSERT(page.Check() == PageMagicKind::UNFORMATTED);
  TEST_ASSERT(!page.Ready());

  Sample probe = {};
  TEST_ASSERT(!page.Latest(&probe));
  TEST_ASSERT(page.TelemetryReader().Head() == 0);
  TEST_ASSERT(page.Region().Seq() == 0);

  // 未格式化也能写：写者只需要映射，Format() 负责清零历史。
  // Writing before Format() is defined: the writer only needs the mapping, and
  // Format() zeroes the history.
  TEST_ASSERT(page.WriteSample(make_sample(0)) == 1);

  page.Format();
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);
  TEST_ASSERT(page.Ready());
  TEST_ASSERT(page.Region().Seq() == 0);
  TEST_ASSERT(!page.Latest(&probe));

  const auto* header = reinterpret_cast<const PageHeader*>(mapping.Data());
  const PageLayout layout = TestPage::Layout();
  TEST_ASSERT(header->magic == PAGE_MAGIC);
  TEST_ASSERT(header->abi_version == layout.abi_version);
  TEST_ASSERT(header->page_size == layout.page_size);
  TEST_ASSERT(header->slot_count == layout.slot_count);
  TEST_ASSERT(header->sample_size == layout.sample_size);
  TEST_ASSERT(header->payload_size == layout.payload_size);
  TEST_ASSERT(header->region_offset == layout.region_offset);
  TEST_ASSERT(header->tag == layout.tag);

  page.WriteSample(make_sample(1));
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
void test_foreign_page(const Mapping& mapping)
{
  const PageHeader foreign = {.magic = 0x11223344U,
                              .abi_version = 1,
                              .page_size = static_cast<uint32_t>(TestFormat::PAGE_SIZE),
                              .slot_count = TestFormat::SLOT_COUNT,
                              .sample_size = sizeof(Sample),
                              .payload_size = sizeof(TestProtocolPayload),
                              .region_offset = 0,
                              .tag = 0};
  std::memcpy(mapping.Data(), bytes_of(foreign).data(), sizeof(foreign));

  TestPage page(mapping.Data());
  TEST_ASSERT(page.Check() == PageMagicKind::FOREIGN);

  page.Format();
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);
}

/// 4. 双端契约漂移：魔术字对、几何/载荷/标签不对，必须报 MISMATCH 而不是静默错读。
///    Contract drift between the two ends: right magic but wrong geometry, payload or
///    tag, must be reported as MISMATCH instead of silently misread.
void test_mismatch_page(const Mapping& mapping)
{
  TestPage page(mapping.Data());
  page.Format();
  TEST_ASSERT(page.Check() == PageMagicKind::FORMATTED);

  const TaggedPage drifted(mapping.Data());
  TEST_ASSERT(drifted.Check() == PageMagicKind::MISMATCH);
  TEST_ASSERT(!drifted.Ready());
  TEST_ASSERT(drifted.TelemetryReader().Head() == 0);

  const SmallPage resized(mapping.Data());
  TEST_ASSERT(resized.Check() == PageMagicKind::MISMATCH);

  // 帧几何漂移同样必须报出来：页头记录几何，不依赖库常量。
  // Frame geometry drift must be reported too: the header records the geometry, no
  // library constants involved.
  const WidePage regeometried(mapping.Data());
  TEST_ASSERT(regeometried.Check() == PageMagicKind::MISMATCH);

  // 与外来页不同：MISMATCH 页可以被这一侧的 Format() 重新认领。
  // Unlike a foreign page: a mismatched page can be reclaimed by this side's
  // Format().
  TaggedPage reclaim(mapping.Data());
  reclaim.Format();
  TEST_ASSERT(reclaim.Check() == PageMagicKind::FORMATTED);
  TEST_ASSERT(page.Check() == PageMagicKind::MISMATCH);
}

/// 5. 发布索引与最新采样。Publish index and the latest sample.
void test_latest(const Mapping& mapping)
{
  TestPage page(mapping.Data());
  for (uint32_t index = 0; index < 3; ++index)
  {
    TEST_ASSERT(page.TelemetryWriter().Write(make_sample(index)) == index + 1);
  }
  TEST_ASSERT(page.TelemetryWriter().Head() == 3);

  Sample latest = {};
  TEST_ASSERT(page.Latest(&latest));
  TEST_ASSERT(same_sample(latest, make_sample(2)));

  const auto* ring = reinterpret_cast<const TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(
      mapping.Data() + telemetry_offset());
  TEST_ASSERT(same_sample(ring->ring[2], make_sample(2)));
  TEST_ASSERT(same_sample(ring->ring[0], make_sample(0)));

  auto* writable_ring = reinterpret_cast<TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(
      mapping.Data() + telemetry_offset());
  writable_ring->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(page.TelemetryWriter().Write(make_sample(3)) == 0);
  writable_ring->write_state.store(0, std::memory_order_release);
}

/// 6. `Since()` 的区间、上界与 gap。Range, bound and gap of `Since()`.
void test_since(const Mapping& mapping)
{
  TestPage page(mapping.Data());
  page.ClearHistory();
  auto writer = page.TelemetryWriter();
  const auto reader = page.TelemetryReader();

  SinceResult result = {};
  std::array<Sample, TestFormat::SLOT_COUNT> out = {};

  auto* telemetry_ring =
      reinterpret_cast<TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(
          mapping.Data() + telemetry_offset());
  telemetry_ring->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::BUSY);
  TEST_ASSERT(result.dropped == 0);
  telemetry_ring->write_state.store(0, std::memory_order_release);

  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 0);

  // 写者跑出 100 条，落后 90 槽的读者整段丢：`head - last > 32`。
  // The writer ran 100 ahead; a reader 90 slots behind drops the whole range.
  for (uint32_t index = 0; index < 100; ++index)
  {
    writer.Write(make_sample(index));
  }
  TEST_ASSERT(writer.Head() == 100);
  TEST_ASSERT(reader.Since(10, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 90);
  TEST_ASSERT(result.next == 100);

  // 正好落后 32 槽仍是可解析区间（上界是 `> SLOT_COUNT`）。
  // Exactly 32 slots behind is still parseable: the bound is `> SLOT_COUNT`.
  page.ClearHistory();
  for (uint32_t index = 0; index < 32; ++index)
  {
    writer.Write(make_sample(index));
  }
  TEST_ASSERT(reader.Since(0, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 32);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 32);
  for (uint32_t index = 0; index < 32; ++index)
  {
    TEST_ASSERT(same_sample(out.at(index), make_sample(index)));
  }

  // 接着 drain 一小段：区间上界是 head，序列正序。
  // Drain a short range: the upper bound is head and the order is ascending.
  for (uint32_t index = 32; index < 34; ++index)
  {
    writer.Write(make_sample(index));
  }
  TEST_ASSERT(reader.Since(32, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 2);
  TEST_ASSERT(result.dropped == 0);
  TEST_ASSERT(result.next == 34);
  TEST_ASSERT(same_sample(out.at(0), make_sample(32)));
  TEST_ASSERT(same_sample(out.at(1), make_sample(33)));

  // 再写一条即可证明回绕：ClearHistory() 后索引从 0 重新计，head 34 落在槽 2。
  // One more write proves the wrap: after ClearHistory() the index restarts at 0, so
  // head 34 lives in slot 2.
  writer.Write(make_sample(34));
  TEST_ASSERT(reader.Since(34, out.data(), out.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 1);
  TEST_ASSERT(result.next == 35);
  TEST_ASSERT(same_sample(out.at(0), make_sample(34)));
  const auto* ring =
      reinterpret_cast<const TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(
          mapping.Data() + telemetry_offset());
  TEST_ASSERT(same_sample(ring->ring[2], make_sample(34)));

  // 空缓冲仍回答区间问题，但没有样本可交付。
  // A null buffer still answers the range question, but delivers no samples.
  TEST_ASSERT(reader.Since(34, nullptr, TestFormat::SLOT_COUNT, &result) ==
              ErrorCode::OK);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped == 1);
  TEST_ASSERT(result.next == 35);

  // 容量小于区间时只交付容量内的一段，其余计入 dropped（调用方缓冲不足，不是历史覆写）。
  // 区间按「最新优先」拷贝，被裁掉的是最旧一端：tiny 里是最新的一条。
  // A capacity smaller than the range delivers only what fits and counts the rest as
  // dropped (caller capacity, not a history overwrite). The range is copied newest
  // first, so clamping drops the oldest end and tiny keeps the newest sample.
  std::array<Sample, 1> tiny = {};
  writer.Write(make_sample(35));
  writer.Write(make_sample(36));
  TEST_ASSERT(reader.Since(35, tiny.data(), tiny.size(), &result) == ErrorCode::OK);
  TEST_ASSERT(result.written == 1);
  TEST_ASSERT(result.dropped == 1);
  TEST_ASSERT(result.next == 37);
  TEST_ASSERT(same_sample(tiny.at(0), make_sample(36)));

  // 上一纪元留下的 last_seen（大于 head）是 gap，不是回绕出来的区间。
  // A last_seen from a previous epoch (larger than head) is a gap, not a wrapped range.
  TEST_ASSERT(reader.Since(1000, out.data(), out.size(), &result) == ErrorCode::EMPTY);
  TEST_ASSERT(result.written == 0);
  TEST_ASSERT(result.dropped > 0);
  TEST_ASSERT(result.next == 37);
}

/// 7. region 的稳定读、失败重试的上限与 BUSY 判定。
///    Stable region reads, the retry budget and the BUSY verdict.
void test_region(const Mapping& mapping)
{
  TestPage page(mapping.Data());
  page.ClearHistory();
  auto reference = page.Region();

  TestProtocolPayload snapshot = {};
  TEST_ASSERT(reference.Read(static_cast<TestProtocolPayload*>(nullptr), nullptr) ==
              ErrorCode::PTR_NULL);
  uint32_t seq = 0;
  TEST_ASSERT(reference.Read(&snapshot, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 0);
  TEST_ASSERT(snapshot.found == 0);

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

  TEST_ASSERT(reference.Read(&snapshot, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 2);
  TEST_ASSERT(snapshot.aim_x == 0.125F);
  TEST_ASSERT(snapshot.aim_y == 0.75F);
  TEST_ASSERT(snapshot.cmd == 0);
  TEST_ASSERT(snapshot.param_id == 0);

  auto* region = reinterpret_cast<Region<TestProtocolPayload>*>(
      mapping.Data() + region_offset<TestFormat, TestProtocolPayload>());
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
  TEST_ASSERT(reference.Read(&snapshot, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 2);
  TEST_ASSERT(snapshot.aim_x == 123.0F);

  // BUSY 只在重试预算内读不到自洽快照时出现：把重试预算设为 0 就等于强制走该分支，
  // 用来钉住「out 只在 OK 时被写」的约定。
  // BUSY appears only when no coherent snapshot fits the retry budget; a budget of 0
  // forces that branch and pins the "out is written only on OK" contract.
  TEST_ASSERT(reference.Read(&snapshot, nullptr, 0) == ErrorCode::BUSY);
  TEST_ASSERT(snapshot.aim_x == 123.0F);

  region->write_state.store(1, std::memory_order_release);
  TEST_ASSERT(reference.Read(&snapshot, nullptr) == ErrorCode::BUSY);
  TEST_ASSERT(snapshot.aim_x == 123.0F);
  TEST_ASSERT(reference.Write(payload) == 0);
  region->write_state.store(0, std::memory_order_release);
}

}  // namespace

int main()
{
  test_layout();
  test_unbound_page();

  {
    const Mapping mapping(TestFormat::PAGE_SIZE);
    test_cold_page_and_index(mapping);
    test_foreign_page(mapping);
    test_mismatch_page(mapping);
    test_latest(mapping);
    test_since(mapping);
    test_region(mapping);
  }

  return 0;
}
