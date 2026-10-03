/**
 * @file test_linux_shared_page.cpp
 * @brief `linux_shared_page.hpp` 的适配器测试 / Adapter tests for
 *        `linux_shared_page.hpp`.
 *
 * 测什么 / Checks:
 *   1. 事件门：没有门铃就没有发布（数据不许自己"被轮询"）；门铃到达才 drain +
 *      发布；空铃/超时/冷页都不发布。
 *   2. drain 区间：整段以一条 `TelemetryBatch` 进真实 LibXR topic 的回调订阅者，
 *      顺序、条数、`head` 与发布时间戳；多次通知 coalesce 成一次 drain。
 *   3. gap：被写者覆写的区间整段丢弃并标记，随后能重新同步。
 *   4. 参考/命令下行：经独立页视图回读，`seq` 递增、字段与单位不变。
 *
 * 运行前提 / Setup: 无。页由匿名映射提供，门铃用 eventfd（与平台绑定同形态），
 * 真实 `Topic` 在本进程内注册回调订阅者，所以不需要 `/dev/mem`、root、
 * `cvi-rtos-cmdqu` 或第二个进程。
 * None. The page is an anonymous mapping, the doorbell is an eventfd (the same
 * shape a platform binding provides), and the real `Topic` registers an
 * in-process callback subscriber, so no `/dev/mem`, root, `cvi-rtos-cmdqu` or
 * second process is needed.
 */

#include <sys/eventfd.h>
#include <sys/mman.h>
#include <unistd.h>

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

/// 一个可敲的门铃：测试自建 eventfd，用 `Doorbell::FromFd` 包成同形态事件源。
/// A ringable doorbell: the test owns the eventfd and wraps it with
/// `Doorbell::FromFd`, the same shape a platform binding provides.
class RingableDoorbell
{
 public:
  RingableDoorbell() : fd_(::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK))
  {
    TEST_ASSERT(fd_ >= 0);
  }

  ~RingableDoorbell() { ::close(fd_); }

  RingableDoorbell(const RingableDoorbell&) = delete;
  RingableDoorbell& operator=(const RingableDoorbell&) = delete;

  [[nodiscard]] Doorbell Wrap() const { return Doorbell::FromFd(fd_, false); }

  void Ring() const
  {
    const uint64_t value = 1;
    ssize_t written = 0;
    do
    {
      written = ::write(fd_, &value, sizeof(value));
    } while (written < 0);
  }

 private:
  int fd_;
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
  const LibXR::Topic topic(
      LibXR::Topic::FindOrCreate<TestBatch>(TELEMETRY_TOPIC_NAME, &domain));
  BatchCapture capture;
  auto callback = LibXR::Topic::Callback::Create(on_batch, &capture);

  RingableDoorbell ringer;
  TestAdapter adapter(ringer.Wrap(), page, topic);
  TEST_ASSERT(adapter.Drain(nullptr) == 0);
  LibXR::Topic& boundary = adapter;
  boundary.RegisterCallback(callback);

  const TestAdapter produced(ringer.Wrap(), page, "shared_page_owned_topic");
  TEST_ASSERT(produced.PayloadSize() == sizeof(TestBatch));

  // 1. 事件门：没有门铃就没有发布——数据写下了也一样。
  // The event gate: no doorbell means no publish, even with data waiting.
  TEST_ASSERT(!adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 0);
  TEST_ASSERT(adapter.LastSeen() == 0);
  auto writer = page.TelemetryWriter();
  writer.Write(make_sample(0));
  TEST_ASSERT(!adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 0);

  // 门铃到达才 drain：期间写下的采样随这次 drain 一起出去。
  // A doorbell drains: the sample written meanwhile rides along with it.
  ringer.Ring();
  TEST_ASSERT(adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 1);
  TEST_ASSERT(adapter.LastSeen() == 1);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(0)));

  // 空铃（没有新数据）不发布；超时是超时，不是发布。
  // A spurious notification publishes nothing; a timeout is a timeout.
  ringer.Ring();
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
  ringer.Ring();
  ringer.Ring();
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
  TestAdapter epoch_adapter(ringer.Wrap(), page, topic);
  epoch_adapter.Drain(nullptr);
  for (uint32_t index = 0; index < 5; ++index)
  {
    writer.Write(make_sample(index));
  }
  ringer.Ring();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 3);
  TEST_ASSERT(capture.last.count == 5);
  TEST_ASSERT(capture.last.head == 5);
  TEST_ASSERT(capture.last.gap == 0);

  for (uint32_t index = 5; index < 80; ++index)
  {
    writer.Write(make_sample(index));
  }
  ringer.Ring();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 4);
  TEST_ASSERT(capture.last.gap == 1);
  TEST_ASSERT(capture.last.count == 0);
  TEST_ASSERT(capture.last.head == 80);
  TEST_ASSERT(epoch_adapter.LastSeen() == 80);

  writer.Write(make_sample(80));
  writer.Write(make_sample(81));
  ringer.Ring();
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
      reinterpret_cast<TelemetryRing<Sample, TestFormat::SLOT_COUNT>*>(mapping.Data() +
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

  // 5. 冷页（未格式化）即使门铃响了也不发布。
  // A cold (unformatted) page publishes nothing even when the doorbell rings.
  const Mapping cold_mapping(TestFormat::PAGE_SIZE);
  const TestPage cold_page(cold_mapping.Data());
  TestAdapter cold_adapter(ringer.Wrap(), cold_page, topic);
  ringer.Ring();
  TEST_ASSERT(!cold_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 5);

  // 6. 写者死在 claim 窗口里：持续 BUSY 超阈值后自动破 claim 重同步，历史不丢。
  // A writer dead inside the claim window: `BUSY` past the threshold breaks the
  // claim and resynchronises, with the history intact.
  writer.Write(make_sample(82));
  telemetry_ring->write_state.store(1, std::memory_order_release);
  ringer.Ring();
  TEST_ASSERT(!epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 5);
  ::usleep(2 * STALE_CLAIM_TIMEOUT_US);
  ringer.Ring();
  TEST_ASSERT(epoch_adapter.WaitAndPublish(0));
  TEST_ASSERT(capture.calls == 6);
  TEST_ASSERT(capture.last.count == 1);
  TEST_ASSERT(same_sample(capture.last.ring[0], make_sample(82)));
  TEST_ASSERT(epoch_adapter.LastSeen() == 83);

  return 0;
}
