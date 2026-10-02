#pragma once

#include <cstddef>
#include <cstdint>

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
struct TelemetryBatch
{
  Sample ring[TELEMETRY_SLOTS] = {};
  uint32_t count = 0;
  uint32_t head = 0;
  uint8_t gap = 0;
  uint8_t reserved[3] = {};
};

inline constexpr size_t TELEMETRY_BATCH_BYTES = sizeof(TelemetryBatch);

static_assert(sizeof(TelemetryBatch) == sizeof(Sample) * TELEMETRY_SLOTS + 16);

/**
 * @brief Linux view of a shared page and its telemetry topic.
 */
class LinuxSharedPage : public SharedPage
{
 public:
  LinuxSharedPage(uint64_t physical_address, Topic topic,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US);
  LinuxSharedPage(uint64_t physical_address,
                  const char* topic_name = TELEMETRY_TOPIC_NAME,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US);
  LinuxSharedPage(const SharedPage& page, Topic topic,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US);
  LinuxSharedPage(const SharedPage& page, const char* topic_name = TELEMETRY_TOPIC_NAME,
                  uint32_t drain_period_us = DEFAULT_DRAIN_PERIOD_US);

  ~LinuxSharedPage() = default;

  LinuxSharedPage(const LinuxSharedPage&) = delete;
  LinuxSharedPage& operator=(const LinuxSharedPage&) = delete;

  /**
   * @brief Drain and publish when the period has elapsed.
   */
  bool Poll(uint64_t now_us = UINT64_MAX);

  /**
   * @brief Drain the next telemetry batch without publishing it.
   */
  uint32_t Drain(TelemetryBatch* batch);

  /**
   * @brief Return the consumed telemetry index.
   */
  [[nodiscard]] uint32_t LastSeen() const;

 private:
  struct Mapping
  {
    static Mapping Open(uint64_t physical_address);

    Mapping() = default;
    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
    Mapping(Mapping&& other) noexcept;
    Mapping& operator=(Mapping&& other) noexcept;
    ~Mapping();

    [[nodiscard]] uint8_t* Data() const { return data; }
    void Reset();

    void* base = nullptr;
    uint8_t* data = nullptr;
    size_t length = 0;
    int fd = -1;
  };

  LinuxSharedPage(Mapping mapping, Topic topic, uint32_t drain_period_us);

  Mapping mapping_;
  uint32_t last_seen_ = 0;
  uint64_t last_drain_us_ = 0;
  uint32_t period_us_ = DEFAULT_DRAIN_PERIOD_US;
  bool clock_started_ = false;
};

}  // namespace LibXR
