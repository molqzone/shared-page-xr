#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "libxr.hpp"

/**
 * @file shared_page.hpp
 * @brief Platform-neutral shared-page contract.
 */

namespace LibXR
{
/// @brief Shared-page size.
inline constexpr size_t PAGE_SIZE = 4096;

/// @brief Page header magic.
inline constexpr uint32_t PAGE_MAGIC = 0x31506461U;  // NOLINT

/// @brief Access-unit slot magic.
inline constexpr uint32_t ACCESS_UNIT_MAGIC = 0x31786F42U;  // NOLINT

/// @brief Telemetry ring capacity.
inline constexpr uint32_t TELEMETRY_SLOTS = 64;

/// @brief Maximum access-unit payload size.
inline constexpr uint32_t MAILBOX_BYTES = 512 * 1024;

/// @brief Bytes available to the higher-layer region payload.
inline constexpr size_t REGION_PAYLOAD_BYTES = 24;

/// @brief Unknown access-unit format.
inline constexpr uint32_t ACCESS_UNIT_FORMAT_UNKNOWN = 0;

/// @brief H.264 Annex-B access-unit format.
inline constexpr uint32_t ACCESS_UNIT_FORMAT_H264_ANNEX_B = 1;

/** @brief One control-loop telemetry sample.
 *
 * 字段单位是硬件原生宽度：IMU/温度为 BMI088（xrobot-org/BMI088）传感器原始 LSB
 * （传感器本体系，不经驱动的量程换算与 rotation），舵机为硬件命令字。
 * Fields use hardware-native widths: IMU/temperature are raw BMI088 LSB (sensor
 * frame, before the driver's scaling and rotation); servos are hardware command
 * words.
 */
struct Sample
{
  uint64_t ticks;  ///< C606 tick（rdtime，25MHz 域）。C606 tick (rdtime, 25 MHz domain).
  int16_t accel[3];          ///< BMI088 加计原始 LSB。Raw accel LSB.
  int16_t gyro[3];           ///< BMI088 陀螺原始 LSB。Raw gyro LSB.
  int16_t temperature;       ///< BMI088 温度原始 LSB。Raw temperature LSB.
  uint16_t servo_target[4];  ///< 舵机硬件命令字（C606 解算输出）。Servo hardware command
                             ///< words, solved on the C606 side.
  uint16_t servo_actual[4];  ///< 舵机实际下发/反馈命令字（与 target 同刻度）。Servo
                             ///< applied/read-back command words, same scale as target.
  uint16_t pad;              ///< 填充到 8B 对齐。Padding to 8-byte alignment.
};

static_assert(sizeof(Sample) == 40, "Sample must be 40B with 4 servo channels");
static_assert(offsetof(Sample, temperature) == 20, "Sample::temperature offset pinned");
static_assert(offsetof(Sample, servo_target) == 22, "Sample::servo_target offset pinned");
static_assert(offsetof(Sample, servo_actual) == 30, "Sample::servo_actual offset pinned");
static_assert(offsetof(Sample, pad) == 38, "Sample::pad offset pinned");

/** @brief Telemetry ring and publication state. */
struct TelemetryRing
{
  Sample ring[TELEMETRY_SLOTS];  ///< 定长历史，写者覆写最旧槽。Fixed history; the writer
                                 ///< overwrites the oldest slot.
  std::atomic<uint32_t> head;  ///< 已发布条数（release store）。Published count (release
                               ///< store).
  std::atomic<uint32_t> write_state;  ///< 0 when stable.
};

static_assert(sizeof(TelemetryRing) == sizeof(Sample) * TELEMETRY_SLOTS + 8,
              "TelemetryRing layout pinned");

/** @brief Result of validating a mapped page. */
enum class PageMagicKind : uint8_t
{
  FORMATTED = 0,
  UNFORMATTED = 1,
  FOREIGN = 2,
};

/** @brief Result data returned by `Telemetry::Since()`. */
struct SinceResult
{
  uint32_t written = 0;
  uint32_t dropped = 0;
  uint32_t next = 0;
};

/** @brief Fixed-size opaque payload region. */
struct Region
{
  uint8_t payload[REGION_PAYLOAD_BYTES] = {};
  std::atomic<uint32_t> write_state;  ///< 0 when stable.
  std::atomic<uint32_t> seq;          ///< Incremented after each write.
};

static_assert(sizeof(Region) == 32, "Region layout pinned");
static_assert(offsetof(Region, write_state) == 24, "Region::write_state offset pinned");
static_assert(offsetof(Region, seq) == 28, "Region::seq offset pinned");
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
              "Region::seq must occupy exactly 4 bytes for the pinned offset to hold");

/** @brief Single-slot latest-frame access-unit mailbox. */
struct AccessUnit
{
  uint32_t magic;
  uint32_t format;
  uint32_t width;
  uint32_t height;
  std::atomic<uint32_t> seq;
  std::atomic<uint32_t> length;
  std::atomic<uint32_t> ready;
  std::atomic<uint32_t> write_state;
  uint8_t payload[MAILBOX_BYTES];

  /// @brief Maximum payload size.
  static constexpr uint32_t MAX_BYTES = MAILBOX_BYTES;

  /// @brief Unknown format.
  static constexpr uint32_t FORMAT_UNKNOWN = ACCESS_UNIT_FORMAT_UNKNOWN;
  /// @brief H.264 Annex-B format.
  static constexpr uint32_t FORMAT_H264_ANNEX_B = ACCESS_UNIT_FORMAT_H264_ANNEX_B;
};

/// @brief In-page telemetry offset.
inline constexpr size_t telemetry_offset() { return 8; }

/// @brief In-page reference offset.
inline constexpr size_t region_offset() { return PAGE_SIZE - sizeof(Region); }

/** @brief Shared-page header. */
struct PageHeader
{
  uint32_t magic;
  uint32_t page_size;
};

static_assert(offsetof(PageHeader, page_size) == 4, "PageHeader layout pinned");
static_assert(telemetry_offset() >= sizeof(PageHeader),
              "telemetry must not overlap header");
static_assert(telemetry_offset() + sizeof(TelemetryRing) <= region_offset(),
              "telemetry ring and region must not overlap");

/** @brief View of the telemetry ring. */
class Telemetry
{
 public:
  Telemetry() = default;

  /// @brief Bind to a page.
  explicit Telemetry(void* addr);

  Telemetry(const Telemetry&) = default;
  Telemetry& operator=(const Telemetry&) = default;
  Telemetry(Telemetry&&) = default;
  Telemetry& operator=(Telemetry&&) = default;

  /// @brief Publish one sample. Returns the new head, or zero when busy.
  uint32_t Write(const Sample& sample);

  /// @brief Return the published count.
  [[nodiscard]] uint32_t Head() const;

  /// @brief Read the latest coherent sample.
  [[nodiscard]] bool Latest(Sample* sample) const;

  /**
   * @brief Copy samples after `last_seen`.
   * @return `OK`, `EMPTY`, `BUSY`, or `PTR_NULL` for an unbound view or null result.
   */
  [[nodiscard]] ErrorCode Since(uint32_t last_seen, Sample* samples, uint32_t capacity,
                                SinceResult* result) const;

  /// @brief Advance a reader to the current head.
  [[nodiscard]] uint32_t SeekToHead() const;

 private:
  std::atomic<uint32_t>* head_ = nullptr;  ///< 页内 `ring.head` 的原子视图。Atomic view
                                           ///< of the in-page `ring.head`.
  std::atomic<uint32_t>* state_ = nullptr;
  Sample* ring_ = nullptr;
};

/** @brief View of the opaque reference region. */
class Reference
{
 public:
  Reference() = default;

  /// @brief Bind to a page.
  explicit Reference(void* addr);

  Reference(const Reference&) = default;
  Reference& operator=(const Reference&) = default;
  Reference(Reference&&) = default;
  Reference& operator=(Reference&&) = default;

  /// @brief Read a coherent region snapshot.
  [[nodiscard]] ErrorCode Read(Region* out, uint32_t retries = 8) const;

  template <typename Payload>
    requires(!std::is_same_v<Payload, Region>)
  [[nodiscard]] ErrorCode Read(Payload* out, uint32_t retries = 8) const
  {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= REGION_PAYLOAD_BYTES);
    if (out == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    Region snapshot = {};
    const ErrorCode result = Read(&snapshot, retries);
    if (result == ErrorCode::OK)
    {
      std::memcpy(out, snapshot.payload, sizeof(Payload));
    }
    return result;
  }

  /// @brief Return the current sequence.
  [[nodiscard]] uint32_t Seq() const;

  /// @brief Write a higher-layer payload. Returns the new sequence, or zero when busy.
  uint32_t Write(const void* payload, size_t size);

  template <typename Payload>
  uint32_t Write(const Payload& payload)
  {
    static_assert(std::is_trivially_copyable_v<Payload>);
    static_assert(sizeof(Payload) <= REGION_PAYLOAD_BYTES);
    return Write(&payload, sizeof(Payload));
  }

  /// @brief Return the in-page region.
  [[nodiscard]] const Region* Raw() const;

 private:
  Region* region_ = nullptr;  ///< 页内参考区。In-page reference region.
};

/** @brief View of one mapped page and its topic boundary. */
class SharedPage : public Topic
{
 public:
  SharedPage() = default;

  /// @brief Bind one mapped page.
  explicit SharedPage(void* addr);
  SharedPage(void* addr, Topic topic);

  [[nodiscard]] bool Valid() const;
  [[nodiscard]] uint8_t* Data() const;
  [[nodiscard]] PageMagicKind Check() const;
  void Format();
  void ClearHistory();

  /// @brief Return whether the page is ready.
  [[nodiscard]] bool Ready() const;

  /// @brief Return the telemetry writer view.
  [[nodiscard]] Telemetry TelemetryWriter();

  /// @brief Return the telemetry view.
  [[nodiscard]] Telemetry TelemetryWriter() const;

  /// @brief Return the telemetry reader view.
  [[nodiscard]] Telemetry TelemetryReader() const;

  /// @brief Return the opaque region view.
  [[nodiscard]] Reference Region();

  /// @brief Return the opaque region view.
  [[nodiscard]] Reference Region() const;

  /// @brief Write one telemetry sample.
  uint32_t WriteSample(const Sample& sample);

  /// @brief Read the latest telemetry sample.
  [[nodiscard]] bool Latest(Sample* sample) const;

 private:
  uint8_t* page_ = nullptr;
};

/** @brief View of the separately mapped access-unit page. */
class AccessUnitPage
{
 public:
  AccessUnitPage() = default;

  /// @brief Bind one mapped page.
  explicit AccessUnitPage(void* addr);

  [[nodiscard]] bool Valid() const;
  [[nodiscard]] uint8_t* Data() const;

  /** @brief Borrowed access-unit view. */
  struct View
  {
    const uint8_t* data = nullptr;
    uint32_t length = 0;
    uint32_t format = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t seq = 0;

    /// @brief Return whether the view contains data.
    [[nodiscard]] bool Valid() const { return data != nullptr && length > 0; }
  };

  /// @brief In-page access-unit offset.
  static constexpr size_t Offset() { return 0; }

  /// @brief Format the access-unit page.
  void Format();

  /// @brief Validate the access-unit page.
  [[nodiscard]] PageMagicKind Check() const;

  /// @brief Publish an access unit. Returns the new sequence, or zero on failure.
  uint32_t Publish(const void* data, uint32_t length, uint32_t format, uint32_t width,
                   uint32_t height);

  /// @brief Acquire the latest borrowed access unit.
  [[nodiscard]] View Acquire(uint32_t retries = 8);

 private:
  [[nodiscard]] AccessUnit* Slot() const;

  uint8_t* page_ = nullptr;
};

}  // namespace LibXR
