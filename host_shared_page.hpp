#pragma once

#include <cstddef>
#include <cstdint>

#include "semaphore.hpp"
#include "shared_page.hpp"

/**
 * @file host_shared_page.hpp
 * @brief Platform-neutral shared-page host adapter with an event-only data path.
 *
 * 平台无关的宿主适配器：**映射与事件源都是注入的**，本文件不接触任何平台 UAPI。
 * 页指针由调用方给出（Linux 侧参考绑定见 `linux_mapping.hpp`，MCU 侧由链接脚本
 * 分配的符号给出），事件源是 LibXR `Semaphore`（每个 LibXR 平台都有实现：Linux
 * futex、FreeRTOS 队列、bare-metal 自旋）。绑定侧在数据到达时 `Post()`（ISR 里
 * `PostFromCallback(in_isr)`），本适配器只负责等、drain、发布。
 *
 * Platform-neutral host adapter: the mapping and the event source are both
 * injected and this header touches no platform UAPI. The caller supplies the
 * page pointer (see `linux_mapping.hpp` for the Linux reference binding, or a
 * linker-script symbol on an MCU) and a LibXR `Semaphore` as the event source
 * (implemented on every LibXR platform: futex on Linux, a queue on FreeRTOS,
 * a spin on bare metal). The binding side posts it when data arrives
 * (`PostFromCallback(in_isr)` from an ISR); this adapter only waits, drains
 * and publishes.
 *
 * 数据到达**只由事件信号量触发**：构造即绑定事件源（空指针即拒绝构造），消费入口
 * 只有 `WaitAndPublish()` 一条路——等信号量、drain、发布。这里刻意不提供轮询路径：
 * 轮询一旦存在就会因为"方便"变成实际使用的那条，事件路径则慢慢烂掉。
 * Data arrival is triggered by the event semaphore only: construction binds
 * the event source (a null pointer is refused) and `WaitAndPublish()` is the
 * single consumption entry -- wait, drain, publish. A polling path is
 * deliberately absent: once one exists it becomes the path actually used
 * because it is convenient, and the event path rots.
 */

namespace LibXR
{

/// @brief Default telemetry topic name.
inline constexpr const char* TELEMETRY_TOPIC_NAME = "telemetry";

/// @brief Block until a notification arrives, matching `Semaphore::Wait`'s own
///        `UINT32_MAX` semantics.
inline constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

/// @brief A `write_state` claim held longer than this means the writer died inside
///        the claim window (the live claim lasts a few stores).
inline constexpr uint64_t STALE_CLAIM_TIMEOUT_US = 10000;

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

/** @brief Event source for the host data path: a LibXR semaphore, not owned here.
 *
 * 等待侧只需要 `Wait(timeout_ms)`；通知侧由平台绑定持有同一个 `Semaphore&` 并
 * `Post()` / `PostFromCallback(in_isr)`。 borrowed 语义：生命周期归调用方。
 * The wait side needs only `Wait(timeout_ms)`; the platform binding holds the
 * same `Semaphore&` and posts it. Borrowed: the caller owns the lifetime.
 */
using EventSource = Semaphore;

/**
 * @brief Platform-neutral view of a shared page and its telemetry topic.
 *
 * 模板参数与 `SharedPage` 一致：`F` 是平台帧格式，`T`/`P` 是应用载荷，`TAG` 是
 * 应用侧 wire 版本标签。
 * Same template contract as `SharedPage`: `F` is the platform frame format,
 * `T`/`P` the application payloads and `TAG` the application-side wire tag.
 */
template <typename F, PagePod T, PagePod P, uint32_t TAG = 0>
class SharedPageHost : public SharedPage<F, T, P, TAG>
{
 public:
  /// @brief The topic payload for this contract: one drained batch.
  using Batch = TelemetryBatch<T, F::SLOT_COUNT>;

  /// @brief Bind a mapped page and the event semaphore guarding its data path.
  ///        A null page pointer or a null event source is refused at construction.
  SharedPageHost(uint8_t* page_addr, EventSource* event, Topic topic)
      : SharedPage<F, T, P, TAG>(page_addr, topic), event_(event)
  {
    // 无页或无事件源 = 拒绝构造：轮询不是合法退路。
    // No page or no event source means refuse construction: polling is not a
    // legal fallback.
    ASSERT(page_addr != nullptr);
    ASSERT(event_ != nullptr);
  }

  /// @brief Bind a mapped page and let the adapter create the telemetry topic.
  SharedPageHost(uint8_t* page_addr, EventSource* event,
                 const char* topic_name = TELEMETRY_TOPIC_NAME)
      : SharedPageHost(page_addr, event, Topic::CreateTopic<Batch>(topic_name))
  {
  }

  /// @brief Bind an existing page view (the typical embedded-side shape).
  SharedPageHost(const SharedPage<F, T, P, TAG>& page, EventSource* event, Topic topic)
      : SharedPage<F, T, P, TAG>(page.Data(), topic), event_(event)
  {
    // 无事件源 = 拒绝构造：轮询不是合法退路。
    // No event source means refuse construction: polling is not a legal fallback.
    ASSERT(event_ != nullptr);
  }

  /// @brief Bind an existing page view and created topic.
  SharedPageHost(const SharedPage<F, T, P, TAG>& page, EventSource* event,
                 const char* topic_name = TELEMETRY_TOPIC_NAME)
      : SharedPageHost(page, event, Topic::CreateTopic<Batch>(topic_name))
  {
  }

  // No destructor: the adapter borrows the page address and the event source
  // and owns nothing (global-lifetime convention).
  SharedPageHost(const SharedPageHost&) = delete;
  SharedPageHost& operator=(const SharedPageHost&) = delete;

  /**
   * @brief The data path: wait for the event semaphore, drain the new range,
   *        publish it.
   * @param timeout_ms Bound for the semaphore wait; `WAIT_FOREVER` blocks until
   *        a notification arrives.
   * @return true when a batch was published; false on timeout, a spurious
   *         notification, or a cold page.
   */
  [[nodiscard]] bool WaitAndPublish(uint32_t timeout_ms)
  {
    if (event_ == nullptr)
    {
      return false;
    }
    if (event_->Wait(timeout_ms) != ErrorCode::OK)
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
   *
   * 写者死在 claim 窗口里时 `Since()` 会永久 `BUSY`；持续 BUSY 超过
   * `STALE_CLAIM_TIMEOUT_US` 即判定写者死亡，破 claim 后当场重试一次。历史不受
   * 影响：未发布完的槽位在 `head` 之外，不会漏出。
   * A writer dying inside the claim window makes `Since()` return `BUSY` forever;
   * `BUSY` held past `STALE_CLAIM_TIMEOUT_US` is taken as a dead writer, the claim
   * is broken and the drain retries once. History survives: the in-flight slot
   * lives beyond `head` and never leaks.
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
    ErrorCode status =
        this->TelemetryReader().Since(last_seen_, batch->ring, F::SLOT_COUNT, &result);
    if (status == ErrorCode::BUSY)
    {
      const uint64_t now_us = static_cast<uint64_t>(Timebase::GetMicroseconds());
      if (!busy_tracking_)
      {
        busy_tracking_ = true;
        busy_since_us_ = now_us;
      }
      else if (now_us - busy_since_us_ > STALE_CLAIM_TIMEOUT_US)
      {
        busy_tracking_ = false;
        this->RecoverStaleClaims();
        status =
            this->TelemetryReader().Since(last_seen_, batch->ring, F::SLOT_COUNT, &result);
      }
      if (status == ErrorCode::BUSY)
      {
        return 0;
      }
    }
    busy_tracking_ = false;

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
  EventSource* event_ = nullptr;
  uint32_t last_seen_ = 0;
  uint64_t busy_since_us_ = 0;
  bool busy_tracking_ = false;
};

}  // namespace LibXR
