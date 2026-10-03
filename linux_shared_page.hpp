#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

#include "shared_page.hpp"

/**
 * @file linux_shared_page.hpp
 * @brief Linux shared-page topic adapter.
 */

namespace LibXR
{

/// @brief Default telemetry topic name.
inline constexpr const char* TELEMETRY_TOPIC_NAME = "telemetry";

/// @brief Default drain period.
inline constexpr uint32_t DEFAULT_DRAIN_PERIOD_US = 1000;

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

  LinuxSharedPage(uint64_t physical_address, Topic topic,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US)
      : LinuxSharedPage(detail::LinuxMapping::Open(physical_address, F::PAGE_SIZE),
                        topic, drain_period_us)
  {
  }

  LinuxSharedPage(uint64_t physical_address,
                  const char* topic_name = TELEMETRY_TOPIC_NAME,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US)
      : LinuxSharedPage(physical_address, Topic::CreateTopic<Batch>(topic_name),
                        drain_period_us)
  {
  }

  LinuxSharedPage(const SharedPage<F, T, P, TAG>& page, Topic topic,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US)
      : SharedPage<F, T, P, TAG>(page.Data(), topic), period_us_(drain_period_us)
  {
    ASSERT(drain_period_us != 0);
  }

  LinuxSharedPage(const SharedPage<F, T, P, TAG>& page,
                  const char* topic_name = TELEMETRY_TOPIC_NAME,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US)
      : LinuxSharedPage(page, Topic::CreateTopic<Batch>(topic_name), drain_period_us)
  {
  }

  ~LinuxSharedPage() = default;

  LinuxSharedPage(const LinuxSharedPage&) = delete;
  LinuxSharedPage& operator=(const LinuxSharedPage&) = delete;

  /**
   * @brief Drain and publish when the period has elapsed.
   */
  bool Poll(uint64_t now_us = UINT64_MAX)
  {
    static_assert(offsetof(Batch, count) == sizeof(T) * F::SLOT_COUNT,
                  "the batch counter must follow the ring without padding");

    if (now_us == UINT64_MAX)
    {
      now_us = static_cast<uint64_t>(Timebase::GetMicroseconds());
    }

    if (!clock_started_)
    {
      clock_started_ = true;
    }
    else if (now_us >= last_drain_us_ && now_us - last_drain_us_ < period_us_)
    {
      return false;
    }
    last_drain_us_ = now_us;

    Batch batch = {};
    if (Drain(&batch) == 0 && batch.gap == 0)
    {
      return false;
    }

    this->Publish(batch, MicrosecondTimestamp(now_us));
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
    const ErrorCode status = this->TelemetryReader().Since(
        last_seen_, batch->ring, F::SLOT_COUNT, &result);
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
  LinuxSharedPage(detail::LinuxMapping mapping, Topic topic, uint32_t drain_period_us)
      : SharedPage<F, T, P, TAG>(mapping.Data(), topic),
        mapping_(std::move(mapping)),
        period_us_(drain_period_us)
  {
    ASSERT(drain_period_us != 0);
  }

  detail::LinuxMapping mapping_;
  uint32_t last_seen_ = 0;
  uint64_t last_drain_us_ = 0;
  uint32_t period_us_ = DEFAULT_DRAIN_PERIOD_US;
  bool clock_started_ = false;
};

}  // namespace LibXR
