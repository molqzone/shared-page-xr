#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

#include "shared_page.hpp"

/**
 * @file linux_shared_page.hpp
 * @brief Linux shared-page topic adapter with an event-only data path.
 *
 * 数据到达**只由门铃事件触发**：构造即绑定事件源（取不到即失败），消费入口只有
 * `WaitAndPublish()` 一条路——等门铃、drain、发布。这里刻意不提供轮询路径：轮询
 * 一旦存在就会因为"方便"变成实际使用的那条，事件路径则慢慢烂掉。
 *
 * Data arrival is triggered by the doorbell event only: construction binds the
 * event source (failing fast without one) and `WaitAndPublish()` is the single
 * consumption entry -- wait, drain, publish. A polling path is deliberately
 * absent: once one exists it becomes the path actually used because it is
 * convenient, and the event path rots.
 */

namespace LibXR
{

/// @brief Default telemetry topic name.
inline constexpr const char* TELEMETRY_TOPIC_NAME = "telemetry";

/**
 * @brief A batch drained from the telemetry ring.
 */
template <PagePod T, uint32_t SLOTS>
struct TelemetryBatch
{
  T ring[SLOTS] = {};
  uint32_t count = 0;
  uint32_t head = 0;
  uint8_t gap = 0;
  uint8_t reserved[3] = {};
};

/** @brief Doorbell event source: the only trigger for data arrival.
 *
 * fd 语义是事件型描述符（eventfd / pipe）：可读即有通知，`Wait()` 消费 8 字节重新
 * 武装。平台绑定把 `cvi-rtos-cmdqu` 之类的设备适配成同一个形态（例如由绑定线程
 * 转发写 eventfd），本库不接触任何平台 UAPI。
 * The fd semantics are an event descriptor (eventfd/pipe): readable means a
 * notification, and `Wait()` consumes 8 bytes to re-arm. Platform bindings adapt
 * devices such as `cvi-rtos-cmdqu` into the same shape (for example a binding
 * thread forwarding into an eventfd); this library touches no platform UAPI.
 */
class Doorbell
{
 public:
  static constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

  /// @brief Create an eventfd-backed doorbell; producers signal by writing 8 bytes.
  static Doorbell EventFd();

  /// @brief Wrap an existing event descriptor. `own` controls close-on-destroy.
  static Doorbell FromFd(int fd, bool own);

  Doorbell() = default;
  ~Doorbell();

  Doorbell(const Doorbell&) = delete;
  Doorbell& operator=(const Doorbell&) = delete;
  Doorbell(Doorbell&& other) noexcept;
  Doorbell& operator=(Doorbell&& other) noexcept;

  [[nodiscard]] bool Valid() const { return fd_ >= 0; }

  /**
   * @brief Block until a notification arrives or the timeout elapses.
   * @return true when a notification was consumed; false on timeout or an
   *         unbound doorbell. `WAIT_FOREVER` blocks indefinitely.
   */
  [[nodiscard]] bool Wait(uint32_t timeout_ms) const;

 private:
  explicit Doorbell(int fd, bool own) : fd_(fd), own_(own) {}

  int fd_ = -1;
  bool own_ = false;
};

namespace detail
{
/** @brief /dev/mem mapping of one shared page. Linux only, see the .cpp. */
class LinuxMapping
{
 public:
  static LinuxMapping Open(uint64_t physical_address, size_t page_bytes);

  LinuxMapping() = default;
  LinuxMapping(const LinuxMapping&) = delete;
  LinuxMapping& operator=(const LinuxMapping&) = delete;
  LinuxMapping(LinuxMapping&& other) noexcept;
  LinuxMapping& operator=(LinuxMapping&& other) noexcept;
  ~LinuxMapping();

  [[nodiscard]] uint8_t* Data() const { return data; }
  void Reset();

  void* base = nullptr;
  uint8_t* data = nullptr;
  size_t length = 0;
  int fd = -1;
};
}  // namespace detail

/**
 * @brief Linux view of a shared page and its telemetry topic.
 */
template <typename F, PagePod T, PagePod P, uint32_t TAG = 0>
class LinuxSharedPage : public SharedPage<F, T, P, TAG>
{
 public:
  /// @brief The topic payload for this contract: one drained batch.
  using Batch = TelemetryBatch<T, F::SLOT_COUNT>;

  LinuxSharedPage(Doorbell doorbell, uint64_t physical_address, Topic topic)
      : LinuxSharedPage(std::move(doorbell),
                        detail::LinuxMapping::Open(physical_address, F::PAGE_SIZE),
                        topic)
  {
  }

  LinuxSharedPage(Doorbell doorbell, uint64_t physical_address,
                  const char* topic_name = TELEMETRY_TOPIC_NAME)
      : LinuxSharedPage(std::move(doorbell), physical_address,
                        Topic::CreateTopic<Batch>(topic_name))
  {
  }

  LinuxSharedPage(Doorbell doorbell, const SharedPage<F, T, P, TAG>& page, Topic topic)
      : SharedPage<F, T, P, TAG>(page.Data(), topic), doorbell_(std::move(doorbell))
  {
    // 无事件源 = 拒绝构造：轮询不是合法退路。
    // No event source means refuse construction: polling is not a legal fallback.
    ASSERT(doorbell_.Valid());
  }

  LinuxSharedPage(Doorbell doorbell, const SharedPage<F, T, P, TAG>& page,
                  const char* topic_name = TELEMETRY_TOPIC_NAME)
      : LinuxSharedPage(std::move(doorbell), page,
                        Topic::CreateTopic<Batch>(topic_name))
  {
  }

  ~LinuxSharedPage() = default;

  LinuxSharedPage(const LinuxSharedPage&) = delete;
  LinuxSharedPage& operator=(const LinuxSharedPage&) = delete;

  /**
   * @brief The data path: wait for a doorbell, drain the new range, publish it.
   * @param timeout_ms Bound for the doorbell wait; `Doorbell::WAIT_FOREVER`
   *        blocks until a notification arrives.
   * @return true when a batch was published; false on timeout, a spurious
   *         notification, or a cold page.
   */
  [[nodiscard]] bool WaitAndPublish(uint32_t timeout_ms)
  {
    if (!doorbell_.Wait(timeout_ms))
    {
      return false;
    }

    Batch batch = {};
    if (Drain(&batch) == 0 && batch.gap == 0)
    {
      return false;
    }

    this->Publish(batch, MicrosecondTimestamp(Timebase::GetMicroseconds()));
    return true;
  }

  /**
   * @brief Drain the next telemetry batch without publishing it.
   */
  uint32_t Drain(Batch* batch)
  {
    if (batch == nullptr)
    {
      return 0;
    }
    *batch = {};

    if (!this->Ready())
    {
      return 0;
    }

    SinceResult result = {};
    const ErrorCode status =
        this->TelemetryReader().Since(last_seen_, batch->ring, F::SLOT_COUNT, &result);
    if (status == ErrorCode::BUSY)
    {
      return 0;
    }

    last_seen_ = result.next;

    batch->count = result.written;
    batch->head = result.next;
    batch->gap = result.dropped > 0 ? 1U : 0U;
    return result.written;
  }

  /**
   * @brief Return the consumed telemetry index.
   */
  [[nodiscard]] uint32_t LastSeen() const { return last_seen_; }

 private:
  LinuxSharedPage(Doorbell doorbell, detail::LinuxMapping mapping, Topic topic)
      : SharedPage<F, T, P, TAG>(mapping.Data(), topic),
        doorbell_(std::move(doorbell)),
        mapping_(std::move(mapping))
  {
    ASSERT(doorbell_.Valid());
  }

  Doorbell doorbell_;
  detail::LinuxMapping mapping_;
  uint32_t last_seen_ = 0;
};

}  // namespace LibXR
