/**
 * @file test_host_shared_page.cpp
 * @brief `host_shared_page.hpp` 的适配器测试 / Adapter tests for
 *        `host_shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 事件门：没有事件信号量就没有发布（数据不许自己"被轮询"）；信号量到达才
 *      drain + 发布；空信号/超时/冷页都不发布。
 *   2. drain 区间：整段以一条 `TelemetryBatch` 进真实 LibXR topic 的回调订阅者，
 *      顺序、条数、`head` 与发布时间戳；多次通知 coalesce 成一次 drain。
 *   3. gap：被写者覆写的区间整段丢弃并标记，随后能重新同步。
 *   4. 参考/命令下行：经独立页视图回读，`seq` 递增、字段与单位不变。
 *
 * 运行前提 / Setup: 无。页是进程内静态缓冲，事件源是 LibXR `Semaphore`（与平台
 * 绑定同形态：绑定侧 `Post()`，本侧 `Wait()`），真实 `Topic` 在本进程内注册回调
 * 订阅者，所以不需要 `/dev/mem`、root、eventfd、`cvi-rtos-cmdqu` 或第二个进程。
 * None. The page is an in-process static buffer and the event source is a LibXR
 * `Semaphore` (the same shape a platform binding uses: it posts, this side
 * waits), with the real `Topic` registering an in-process callback subscriber,
 * so no `/dev/mem`, root, eventfd, `cvi-rtos-cmdqu` or second process is needed.
 */

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "host_shared_page.hpp"
#include "libxr_def.hpp"
#include "sample.hpp"
#include "semaphore.hpp"
#include "shared_page.hpp"
#include "test_assert.hpp"
#include "thread.hpp"
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
using TestAdapter = SharedPageHost<TestFormat, Sample, ProtocolPayload>;

/// 收集遥测 topic 上的每一组。Captures every batch published on the telemetry topic.
struct BatchCapture
{
  uint32_t calls = 0;
  TestBatch last = {};
  std::array<uint32_t, 8> counts = {};
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
  ++capture->calls;
}

/// 页存储：进程内静态缓冲，注入给页视图。注入式接缝让测试不需要任何映射调用。
/// The page storage: an in-process static buffer injected into the page views.
/// The injected seam keeps the test free of any mapping call.
alignas(64) uint8_t g_page_storage[TestFormat::PAGE_SIZE];
alignas(64) uint8_t g_cold_page_storage[TestFormat::PAGE_SIZE];

}  // namespace

int main()
{
  static_assert(std::is_base_of_v<LibXR::Topic, TestPage>);
  static_assert(std::is_base_of_v<LibXR::Topic, TestAdapter>);

  TestPage page(g_page_storage);
  page.Format();
  TEST_ASSERT(page.Ready());

  LibXR::Topic::Domain domain("shared_page_xr");
  const LibXR::Topic topic(
      LibXR::Topic::FindOrCreate<TestBatch>(TELEMETRY_TOPIC_NAME, &domain));
  BatchCapture capture;
  auto callback = LibXR::Topic::Callback::Create(on_batch, &capture);

  // 事件源：平台绑定持有同一个 Semaphore 并 Post，本侧只 Wait。
  // The event source: the platform binding holds the same Semaphore and posts
  // it; this side only waits.
  Semaphore event(0);
  TestAdapter adapter(page, &event, topic);
  TEST_ASSERT(adapter.Drain(nullptr) == 0);
  LibXR::Topic& boundary = adapter;
  boundary.RegisterCallback(callback);

  const TestAdapter produced(page, &event, "shared_page_owned_topic");
  TEST_ASSERT(produced.PayloadSize() == sizeof(TestBatch));

  // 1. 事件门：没有事件就没有发布——数据写下了也一样。
  // The event gate: no event means no publish, even with data waiting.
  TEST_ASSERT(!adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 0);
  TEST_ASSERT(adapter.LastSeen() == 0);
  auto writer = page.TelemetryWriter();
  writer.Write(make_sample(0));
  TEST_ASSERT(!adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 0);

  // 事件到达才 drain：期间写下的采样随这次 drain 一起出去。
  // An event drains: the sample written meanwhile rides along with it.
  event.Post();
  TEST_ASSERT(adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 1);
  TEST_ASSERT(adapter.LastSeen() == 1);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(0)));

  // 空信号（没有新数据）不发布；超时是超时，不是发布。
  // A spurious notification publishes nothing; a timeout is a timeout.
  event.Post();
  TEST_ASSERT(!adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 1);
  TEST_ASSERT(!adapter.WaitAndPublish(5));
  TEST_ASSERT(capture.calls == 1);

  // 2. 多次通知 coalesce 成一次 drain：区间整段、正序、head 作索引。
  // Multiple notifications coalesce into one drain: the whole range, ascending,
  // with head as its index.
  for (uint32_t index = 1; index < 4; ++index)
  {
    writer.Write(make_sample(index));
  }
  event.Post();
  event.Post();
  TEST_ASSERT(adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 2);
  TEST_ASSERT(capture.last.count == 3);
  TEST_ASSERT(capture.last.head == 4);
  TEST_ASSERT(capture.last.gap == 0);
  for (uint32_t index = 0; index < 3; ++index)
  {
    TEST_ASSERT(same_sample(capture.last.ring[index], make_sample(index + 1)));
  }

  // 3. gap：被写者跑过的区间整段丢弃并标记，随后重新同步。ClearHistory() 开新纪元，
  // 重建适配器而不是带旧 last_seen 过界。
  // A lapped reader is reported as a gap with no samples; after it the adapter
  // resynchronises. ClearHistory() starts a new epoch, so the adapter is rebuilt
  // rather than carried across the reset.
  page.ClearHistory();
  TestAdapter epoch_adapter(page, &event, topic);
  epoch_adapter.Drain(nullptr);
  for (uint32_t index = 0; index < 5; ++index)
  {
    writer.Write(make_sample(index));
  }
  event.Post();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 3);
  TEST_ASSERT(capture.last.count == 5);
  TEST_ASSERT(capture.last.head == 5);
  TEST_ASSERT(capture.last.gap == 0);

  for (uint32_t index = 5; index < 80; ++index)
  {
    writer.Write(make_sample(index));
  }
  event.Post();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 4);
  TEST_ASSERT(capture.last.gap == 1);
  TEST_ASSERT(capture.last.count == 0);
  TEST_ASSERT(capture.last.head == 80);
  TEST_ASSERT(epoch_adapter.LastSeen() == 80);

  writer.Write(make_sample(80));
  writer.Write(make_sample(81));
  event.Post();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 5);
  TEST_ASSERT(capture.last.gap == 0);
  TEST_ASSERT(capture.last.count == 2);
  TEST_ASSERT(capture.last.head == 82);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(80)));
  TEST_ASSERT(same_sample(capture.last.ring[1], make_sample(81)));

  // 写者活动中的区间不交付、不前进 last_seen。
  // While the writer is active the range is not delivered and last_seen holds.
  auto* telemetry_ring =
      reinterpret_cast<TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(g_page_storage +
                                                                      telemetry_offset());
  telemetry_ring->write_state.store(1, std::memory_order_release);
  TestBatch busy_batch = {};
  TEST_ASSERT(epoch_adapter.Drain(&busy_batch) == 0);
  TEST_ASSERT(epoch_adapter.LastSeen() == 82);
  telemetry_ring->write_state.store(0, std::memory_order_release);
  // 释放后再 drain 一次：走非 BUSY 路径，把卡死跟踪器复位成确定状态。
  // One more drain after the release: the non-BUSY path resets the stale-claim
  // tracker to a deterministic state.
  TEST_ASSERT(epoch_adapter.Drain(&busy_batch) == 0);

  // 4. 参考/命令下行：独立页视图回读，seq 递增。
  // The reference/command downlink: read back through an independent page view,
  // with seq advancing.
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

  TestPage c606_view(g_page_storage);
  uint32_t seq = 0;
  ProtocolPayload decoded = {};
  TEST_ASSERT(c606_view.Region().Read(&decoded, &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 2);
  TEST_ASSERT(decoded.cmd == ProtocolPayload::CMD_PARAM);
  TEST_ASSERT(decoded.param_id == 7);
  TEST_ASSERT(decoded.value == 3.5F);
  TEST_ASSERT(decoded.found == 0);
  TEST_ASSERT(decoded.aim_x == 0.0F);

  // 5. 冷页（未格式化）即使事件到达也不发布。
  // A cold (unformatted) page publishes nothing even when the event is posted.
  const TestPage cold_page(g_cold_page_storage);
  TestAdapter cold_adapter(cold_page, &event, topic);
  event.Post();
  TEST_ASSERT(!cold_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 5);

  // 6. 写者死在 claim 窗口里：持续 BUSY 超阈值后自动破 claim 重同步，历史不丢。
  // A writer dead inside the claim window: `BUSY` past the threshold breaks the
  // claim and resynchronises, with the history intact.
  writer.Write(make_sample(82));
  telemetry_ring->write_state.store(1, std::memory_order_release);
  event.Post();
  TEST_ASSERT(!epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 5);
  Thread::Sleep((2 * STALE_CLAIM_TIMEOUT_US + 999) / 1000);
  event.Post();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 6);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(82)));
  TEST_ASSERT(epoch_adapter.LastSeen() == 83);

  return 0;
}
