/**
 * @file test_linux_shared_page.cpp
 * @brief `linux_shared_page.hpp` 的适配器测试 / Adapter tests for
 *        `linux_shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 节律门：未到周期是早退，到周期才发布。
 *   2. drain 区间：整段以一条 `TelemetryBatch` 进真实 LibXR topic 的回调订阅者，
 *      顺序、条数、`head` 与发布时间戳。
 *   3. gap：被写者覆写的区间整段丢弃并标记，随后能重新同步。
 *   4. 参考/命令下行：经独立页视图回读，`seq` 递增、字段与单位不变。
 *   5. 冷页不产生消息。
 *
 * 运行前提 / Setup: 无。页由匿名映射提供，真实 `Topic` 在本进程内注册回调订阅者，
 * 所以不需要 `/dev/mem`、root 或第二个进程。
 * None. The page is an anonymous mapping and the real `Topic` registers an in-process
 * callback subscriber, so no `/dev/mem`, root or second process is needed.
 *
 * 时钟由测试注入固定的 `now_us`，因此节律判定是确定的，不依赖真实时间。
 * The test injects a fixed `now_us`, so the cadence decision is deterministic and does
 * not depend on wall time.
 */

#include <sys/mman.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "libxr_def.hpp"
#include "linux_shared_page.hpp"
#include "sample.hpp"
#include "shared_page.hpp"
#include "test_assert.hpp"
#include "topic.hpp"

using namespace LibXR;
using LibXRTest::make_sample;
using LibXRTest::same_sample;
using Sample = LibXRTest::Sample;

namespace
{

struct ProtocolPayload
{
  uint8_t found = 0;
  uint8_t pad[3] = {};
  float aim_x = 0;
  float aim_y = 0;
  uint8_t cmd = 0;
  uint8_t pad2[3] = {};
  uint16_t param_id = 0;
  float value = 0;

  static constexpr uint8_t CMD_PARAM = 2;
};

static_assert(sizeof(ProtocolPayload) == 24);

using TestFormat = LibXRTest::Format;
using TestPage = SharedPage<TestFormat, Sample, ProtocolPayload>;
using TestBatch = TelemetryBatch<Sample, TestFormat::SLOT_COUNT>;
using TestAdapter = LinuxSharedPage<TestFormat, Sample, ProtocolPayload>;

/// 收集遥测 topic 上的每一组。Captures every batch published on the telemetry topic.
struct BatchCapture
{
  uint32_t calls = 0;
  TestBatch last = {};
  std::array<uint32_t, 8> counts = {};
  std::array<uint64_t, 8> timestamps_us = {};
};

void on_batch(bool, BatchCapture* capture,
              const LibXR::Topic::MessageView<TestBatch>& message)
{
  if (capture == nullptr || message.data == nullptr)
  {
    return;
  }
  TEST_ASSERT(capture->calls < capture->counts.size());
  capture->last = *message.data;
  capture->counts.at(capture->calls) = message.data->count;
  capture->timestamps_us.at(capture->calls) = static_cast<uint64_t>(message.timestamp);
  ++capture->calls;
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

}  // namespace

int main()
{
  static_assert(std::is_base_of_v<LibXR::Topic, TestPage>);
  static_assert(std::is_base_of_v<LibXR::Topic, TestAdapter>);

  const Mapping mapping(TestFormat::PAGE_SIZE);
  TestPage page(mapping.Data());
  page.Format();
  TEST_ASSERT(page.Ready());

  LibXR::Topic::Domain domain("shared_page_xr");
  const LibXR::Topic topic(LibXR::Topic::FindOrCreate<TestBatch>(TELEMETRY_TOPIC_NAME,
                                                                 &domain));
  BatchCapture capture;
  auto callback = LibXR::Topic::Callback::Create(on_batch, &capture);

  TestAdapter adapter(page, topic, 1000);
  TEST_ASSERT(adapter.Drain(nullptr) == 0);
  LibXR::Topic& boundary = adapter;
  boundary.RegisterCallback(callback);

  const TestAdapter produced(page, "shared_page_owned_topic");
  TEST_ASSERT(produced.PayloadSize() == sizeof(TestBatch));

  // 第一次 Poll 只确立节律基准；C906L 侧还没写数据，所以不发布。
  // The first poll only establishes the cadence baseline; nothing has been written on
  // the C906L side, so it publishes nothing.
  adapter.Poll(1000000);
  TEST_ASSERT(capture.calls == 0);
  TEST_ASSERT(adapter.LastSeen() == 0);

  // 到周期才 drain，期间写下的采样随这次 drain 一起出去。
  // A drain happens once the period elapsed and the sample written meanwhile rides
  // along with it.
  auto writer = page.TelemetryWriter();
  writer.Write(make_sample(0));
  adapter.Poll(1001000);
  TEST_ASSERT(capture.calls == 1);
  TEST_ASSERT(adapter.LastSeen() == 1);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(0)));

  // 差 1us 到周期：早退，不发布。
  // One microsecond short of the period is an early return, not a publish.
  writer.Write(make_sample(1));
  adapter.Poll(1001999);
  TEST_ASSERT(capture.calls == 1);
  adapter.Poll(1002000);
  TEST_ASSERT(capture.calls == 2);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(1)));

  // 新区间整段以一条消息发出，正序，head 作为它的索引。ClearHistory() 开启新纪元
  // （head 从 0 重新计），所以重建适配器而不是把它带过重置点：带着旧 last_seen 跨
  // 重启正是上面 gap 规则覆盖的情形。
  // A new range goes out as one message in ascending order with head as its index.
  // ClearHistory() starts a new epoch, so the adapter is rebuilt rather than carried
  // across the reset: a stale last_seen across a restart is the case the gap rule
  // covers above.
  page.ClearHistory();
  TestAdapter epoch_adapter(page, topic);
  epoch_adapter.Poll(1003000);
  TEST_ASSERT(capture.calls == 2);
  for (uint32_t index = 0; index < 5; ++index)
  {
    writer.Write(make_sample(index));
  }
  epoch_adapter.Poll(1004000);
  TEST_ASSERT(capture.calls == 3);
  TEST_ASSERT(capture.last.count == 5);
  TEST_ASSERT(capture.last.head == 5);
  TEST_ASSERT(capture.last.gap == 0);
  for (uint32_t index = 0; index < 5; ++index)
  {
    TEST_ASSERT(same_sample(capture.last.ring[index], make_sample(index)));
  }

  // 发布时间戳就是调用者的 drain 时刻，下游 metadata 据此与视频帧对齐到同一时钟。
  // The publish timestamp is the caller's drain instant, so downstream metadata can
  // place the range on the same clock as the frames.
  TEST_ASSERT(capture.timestamps_us.at(2) == 1004000);

  // 读者被写者跑过：整段丢弃并标记 gap，而不是编造区间。
  // A lapped reader is reported as a gap with no samples instead of an invented range.
  for (uint32_t index = 5; index < 80; ++index)
  {
    writer.Write(make_sample(index));
  }
  epoch_adapter.Poll(1005000);
  TEST_ASSERT(capture.calls == 4);
  TEST_ASSERT(capture.last.gap == 1);
  TEST_ASSERT(capture.last.count == 0);
  TEST_ASSERT(capture.last.head == 80);
  TEST_ASSERT(epoch_adapter.LastSeen() == 80);

  // gap 之后重新同步：下一段区间又是完整的。
  // After a gap the adapter resynchronises: the next range is complete again.
  writer.Write(make_sample(80));
  writer.Write(make_sample(81));
  epoch_adapter.Poll(1006000);
  TEST_ASSERT(capture.calls == 5);
  TEST_ASSERT(capture.last.gap == 0);
  TEST_ASSERT(capture.last.count == 2);
  TEST_ASSERT(capture.last.head == 82);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(80)));
  TEST_ASSERT(same_sample(capture.last.ring[1], make_sample(81)));

  auto* telemetry_ring =
      reinterpret_cast<TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(
          mapping.Data() + telemetry_offset());
  telemetry_ring->write_state.store(1, std::memory_order_release);
  TestBatch busy_batch = {};
  TEST_ASSERT(epoch_adapter.Drain(&busy_batch) == 0);
  TEST_ASSERT(epoch_adapter.LastSeen() == 82);
  telemetry_ring->write_state.store(0, std::memory_order_release);

  ProtocolPayload payload = {};
  payload.found = 1;
  payload.aim_x = 0.5F;
  payload.aim_y = -0.25F;
  TEST_ASSERT(epoch_adapter.Region().Write(payload) == 1);

  payload = {};
  payload.cmd = ProtocolPayload::CMD_PARAM;
  payload.param_id = 7;
  payload.value = 3.5F;
  TEST_ASSERT(epoch_adapter.Region().Write(payload) == 2);

  TestPage c606_view(mapping.Data());
  uint32_t seq = 0;
  ProtocolPayload decoded = {};
  TEST_ASSERT(c606_view.Region().Read(&decoded, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 2);
  TEST_ASSERT(decoded.cmd == ProtocolPayload::CMD_PARAM);
  TEST_ASSERT(decoded.param_id == 7);
  TEST_ASSERT(decoded.value == 3.5F);
  TEST_ASSERT(decoded.found == 0);
  TEST_ASSERT(decoded.aim_x == 0.0F);

  // aim 与命令共用一块 region，后写者覆盖前写者：这是「一次事件一次 region 写」的
  // 约定，不是丢更新。
  // The aim and the command share one region, so the last writer wins: that is the
  // one-region-write-per-event contract, not a lost update.
  payload = {};
  TEST_ASSERT(epoch_adapter.Region().Write(payload) == 3);
  TEST_ASSERT(c606_view.Region().Read(&decoded, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 3);
  TEST_ASSERT(decoded.found == 0);
  TEST_ASSERT(decoded.cmd == 0);

  // 冷页不产生消息。A cold page produces no message.
  page.ClearHistory();
  const TestPage blank_page(mapping.Data());
  TestAdapter blank_adapter(blank_page, topic, 1000);
  blank_adapter.Poll(2000000);
  TEST_ASSERT(capture.calls == 5);

  TestAdapter rollback_adapter(page, topic, 1000);
  rollback_adapter.Poll(5000);
  writer.Write(make_sample(82));
  TEST_ASSERT(rollback_adapter.Poll(4000));
  TEST_ASSERT(capture.calls == 6);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(82)));

  return 0;
}
