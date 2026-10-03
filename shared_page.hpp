#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "libxr.hpp"

/**
 * @file shared_page.hpp
 * @brief Platform-neutral shared-page protocol with application-supplied wire types.
 *
 * 本库只拥有**协议**：4 KiB 页、页头、64 槽遥测环、`head`/`seq`/`write_state` 的同步
 * 语义与 gap 规则。穿过页面的两个数据结构——遥测采样与参考区 payload——由应用以
 * POD 类型在编译期注入（`SharedPage<SampleT, PayloadT, TAG>`），布局在编译期检查、
 * 在 `Format()`/`Check()` 时以指纹互验。
 * This library owns only the protocol: the 4 KiB page, the header, the 64-slot
 * telemetry ring, the `head`/`seq`/`write_state` synchronization and the gap rule.
 * The two data structures crossing the page -- the telemetry sample and the region
 * payload -- are injected by the application as POD types at compile time
 * (`SharedPage<SampleT, PayloadT, TAG>`); their layout is checked at compile time and
 * cross-verified through a fingerprint at `Format()`/`Check()` time.
 */

namespace LibXR
{
/// @brief Shared-page size, fixed by the physical mapping.
inline constexpr size_t PAGE_SIZE = 4096;

/// @brief Page header magic.
inline constexpr uint32_t PAGE_MAGIC = 0x31506461U;  // NOLINT

/// @brief Telemetry ring capacity. Part of the protocol: the gap bound is 64 slots.
inline constexpr uint32_t TELEMETRY_SLOTS = 64;

/// @brief In-page telemetry offset (header plus alignment padding).
inline constexpr size_t telemetry_offset() { return 16; }

/** @brief Result of validating a mapped page.
 *
 * `MISMATCH` 区别于 `FOREIGN`：页头魔术字与页长正确，但布局指纹对不上，说明双端
 * 用了不同版本的 wire 结构（例如固件热替换后的新旧配对）。
 * `MISMATCH` differs from `FOREIGN`: the magic and page size are right but the layout
 * fingerprint does not match, meaning the two ends were built against different wire
 * structures (for example a stale pairing after a runtime firmware replacement).
 */
enum class PageMagicKind : uint8_t
{
  FORMATTED = 0,
  UNFORMATTED = 1,
  FOREIGN = 2,
  MISMATCH = 3,
};

/** @brief Result data returned by `Telemetry::Since()`. */
struct SinceResult
{
  uint32_t written = 0;
  uint32_t dropped = 0;
  uint32_t next = 0;
};

/** @brief Shared-page header. The layout fingerprint lives in `layout`. */
struct PageHeader
{
  uint32_t magic;
  uint32_t page_size;
  uint32_t layout;
  uint32_t reserved;
};

static_assert(sizeof(PageHeader) == 16, "PageHeader layout pinned");
static_assert(offsetof(PageHeader, page_size) == 4, "PageHeader layout pinned");
static_assert(offsetof(PageHeader, layout) == 8, "PageHeader layout pinned");
static_assert(telemetry_offset() >= sizeof(PageHeader),
              "telemetry must not overlap header");

/** @brief Constraint on a wire type crossing the page.
 *
 * 必须是可按字节拷贝的普通结构：跨核搬运没有构造/析构可言，字段布局由应用用
 * `static_assert` 自行钉住。
 * It must be a plain byte-copyable structure: nothing is constructed or destroyed
 * across the cores, and the application pins its field layout with its own
 * `static_assert`s.
 */
template <typename T>
concept PagePod =
    std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T> &&
    std::is_object_v<T> && !std::is_array_v<T> && !std::is_const_v<T> &&
    !std::is_volatile_v<T>;

/** @brief Telemetry ring and publication state for one sample type. */
template <PagePod T>
struct TelemetryRing
{
  T ring[TELEMETRY_SLOTS];  ///< 定长历史，写者覆写最旧槽。Fixed history; the writer
                            ///< overwrites the oldest slot.
  std::atomic<uint32_t> head;  ///< 已发布条数（release store）。Published count (release
                               ///< store).
  std::atomic<uint32_t> write_state;  ///< 0 when stable.
};

/** @brief Fixed-size payload region for one payload type. */
template <PagePod P>
struct Region
{
  P payload = {};
  std::atomic<uint32_t> write_state;  ///< 0 when stable.
  std::atomic<uint32_t> seq;          ///< Incremented after each write.
};

/// @brief In-page reference offset for one payload type.
template <PagePod P>
inline constexpr size_t region_offset()
{
  return PAGE_SIZE - sizeof(Region<P>);
}

/** @brief Compile-time layout of one contract instantiation.
 *
 * 双端各自从同一对 wire 类型算出这份描述；页头里的 `fingerprint` 是它的摘要，供
 * `Check()` 互验。
 * Each end computes this description from the same pair of wire types; the header's
 * `fingerprint` is its digest, cross-checked by `Check()`.
 */
struct PageLayout
{
  uint32_t sample_size = 0;
  uint32_t payload_size = 0;
  uint32_t region_offset = 0;
  uint32_t fingerprint = 0;
};

namespace detail
{
constexpr uint32_t FNV_OFFSET_BASIS = 2166136261U;  // NOLINT
constexpr uint32_t FNV_PRIME = 16777619U;           // NOLINT

constexpr uint32_t FnvMix(uint32_t hash, uint32_t value)
{
  for (uint32_t shift = 0; shift < 32; shift += 8)
  {
    hash ^= (value >> shift) & 0xFFU;
    hash *= FNV_PRIME;
  }
  return hash;
}

/** @brief Compute and validate the layout of `SharedPage<T, P, TAG>`. */
template <PagePod T, PagePod P, uint32_t TAG = 0>
constexpr PageLayout MakeLayout()
{
  static_assert(sizeof(T) % sizeof(uint32_t) == 0,
                "the sample type must be a multiple of 4 bytes: ring copies are "
                "word-atomic");
  static_assert(sizeof(P) % sizeof(uint32_t) == 0,
                "the payload type must be a multiple of 4 bytes: region copies are "
                "word-atomic");
  static_assert(telemetry_offset() % sizeof(uint32_t) == 0 &&
                    (sizeof(T) * TELEMETRY_SLOTS) % sizeof(uint32_t) == 0,
                "every ring slot must start 4-byte aligned inside the page");
  static_assert(region_offset<P>() % sizeof(uint32_t) == 0,
                "the region must start 4-byte aligned inside the page");
  static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
                "the in-page counters must occupy exactly 4 bytes");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "the in-page counters must be lock-free");
  static_assert(offsetof(TelemetryRing<T>, head) == sizeof(T) * TELEMETRY_SLOTS,
                "head must follow the ring without padding");
  static_assert(offsetof(Region<P>, write_state) == sizeof(P),
                "write_state must follow the payload without padding");
  static_assert(offsetof(Region<P>, seq) == sizeof(P) + sizeof(uint32_t),
                "seq must follow write_state");
  static_assert(telemetry_offset() + sizeof(TelemetryRing<T>) <= region_offset<P>(),
                "the telemetry ring and the region must not overlap");

  uint32_t hash = FNV_OFFSET_BASIS;
  hash = FnvMix(hash, static_cast<uint32_t>(sizeof(T)));
  hash = FnvMix(hash, static_cast<uint32_t>(alignof(T)));
  hash = FnvMix(hash, static_cast<uint32_t>(sizeof(P)));
  hash = FnvMix(hash, static_cast<uint32_t>(alignof(P)));
  hash = FnvMix(hash, TELEMETRY_SLOTS);
  hash = FnvMix(hash, static_cast<uint32_t>(PAGE_SIZE));
  hash = FnvMix(hash, TAG);
  return PageLayout{static_cast<uint32_t>(sizeof(T)), static_cast<uint32_t>(sizeof(P)),
                    static_cast<uint32_t>(region_offset<P>()), hash};
}

/** @brief Type-erased view of the telemetry ring. Protocol engine, see the .cpp. */
class TelemetryCore
{
 public:
  TelemetryCore() = default;
  TelemetryCore(void* addr, const PageLayout& layout);

  uint32_t Write(const void* sample);
  [[nodiscard]] uint32_t Head() const;
  [[nodiscard]] bool Latest(void* out) const;
  [[nodiscard]] ErrorCode Since(uint32_t last_seen, void* out, uint32_t capacity,
                                SinceResult* result) const;


 private:
  std::atomic<uint32_t>* head_ = nullptr;
  std::atomic<uint32_t>* state_ = nullptr;
  uint8_t* ring_ = nullptr;
  uint32_t stride_ = 0;
};

/** @brief Type-erased view of the region. Protocol engine, see the .cpp. */
class ReferenceCore
{
 public:
  ReferenceCore() = default;
  ReferenceCore(void* addr, const PageLayout& layout);

  [[nodiscard]] ErrorCode Read(void* out, uint32_t* seq, uint32_t retries) const;
  uint32_t Write(const void* payload);
  [[nodiscard]] uint32_t Seq() const;
  [[nodiscard]] const void* Raw() const;

 private:
  uint8_t* payload_ = nullptr;
  std::atomic<uint32_t>* state_ = nullptr;
  std::atomic<uint32_t>* seq_ = nullptr;
  uint32_t payload_size_ = 0;
};

/** @brief Type-erased page header operations. Protocol engine, see the .cpp. */
class PageCore
{
 public:
  PageCore() = default;
  PageCore(void* addr, const PageLayout& layout);

  [[nodiscard]] bool Valid() const;
  [[nodiscard]] uint8_t* Data() const;
  [[nodiscard]] PageMagicKind Check() const;
  void Format();
  void ClearHistory();

 private:
  uint8_t* page_ = nullptr;
  PageLayout layout_ = {};
};
}  // namespace detail

/** @brief Typed view of the telemetry ring for one sample type. */
template <PagePod T>
class Telemetry
{
 public:
  Telemetry() = default;

  /// @brief Bind to a page.
  Telemetry(void* addr, const PageLayout& layout) : core_(addr, layout) {}

  Telemetry(const Telemetry&) = default;
  Telemetry& operator=(const Telemetry&) = default;
  Telemetry(Telemetry&&) = default;
  Telemetry& operator=(Telemetry&&) = default;

  /// @brief Publish one sample. Returns the new head, or zero when busy.
  uint32_t Write(const T& sample) { return core_.Write(&sample); }

  /// @brief Return the published count.
  [[nodiscard]] uint32_t Head() const { return core_.Head(); }

  /// @brief Read the latest coherent sample. `sample` is written only on success.
  [[nodiscard]] bool Latest(T* sample) const
  {
    if (sample == nullptr)
    {
      return false;
    }
    T snapshot = {};
    if (!core_.Latest(&snapshot))
    {
      return false;
    }
    *sample = snapshot;
    return true;
  }

  /**
   * @brief Copy samples after `last_seen`.
   * @return `OK`, `EMPTY`, `BUSY`, or `PTR_NULL` for an unbound view or null result.
   */
  [[nodiscard]] ErrorCode Since(uint32_t last_seen, T* samples, uint32_t capacity,
                                SinceResult* result) const
  {
    return core_.Since(last_seen, samples, capacity, result);
  }

  /// @brief Advance a reader to the current head.
  [[nodiscard]] uint32_t SeekToHead() const { return core_.Head(); }

 private:
  detail::TelemetryCore core_;
};

/** @brief Typed view of the region for one payload type. */
template <PagePod P>
class Reference
{
 public:
  Reference() = default;

  /// @brief Bind to a page.
  Reference(void* addr, const PageLayout& layout) : core_(addr, layout) {}

  Reference(const Reference&) = default;
  Reference& operator=(const Reference&) = default;
  Reference(Reference&&) = default;
  Reference& operator=(Reference&&) = default;

  /**
   * @brief Read a coherent payload snapshot. `out` is written only on `OK`.
   * @param seq Receives the observed sequence of the returned payload; may be null.
   * @param retries Retry budget for the coherent-snapshot check.
   * @return `OK`, `BUSY` when no coherent snapshot fits the retry budget, or
   *         `PTR_NULL` for an unbound view or null result.
   *
   * 单一入口、`seq` 必填形参：不提供 `Read(out, retries)` 重载，否则 `Read(out, 0)`
   * 里的 0 会以空指针常量落到 `seq`，静默改变含义。
   * One entry point with a required `seq` parameter: no `Read(out, retries)` overload,
   * otherwise the 0 in `Read(out, 0)` would land on `seq` as a null pointer constant
   * and silently change the meaning.
   */
  [[nodiscard]] ErrorCode Read(P* out, uint32_t* seq, uint32_t retries = 8) const
  {
    if (out == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }
    P snapshot = {};
    const ErrorCode result = core_.Read(&snapshot, seq, retries);
    if (result == ErrorCode::OK)
    {
      *out = snapshot;
    }
    return result;
  }

  /// @brief Return the current sequence.
  [[nodiscard]] uint32_t Seq() const { return core_.Seq(); }

  /// @brief Write one payload. Returns the new sequence, or zero when busy.
  uint32_t Write(const P& payload) { return core_.Write(&payload); }

  /// @brief Return the in-page region frame.
  [[nodiscard]] const Region<P>* Raw() const
  {
    return reinterpret_cast<const Region<P>*>(core_.Raw());
  }

 private:
  detail::ReferenceCore core_;
};

/** @brief Typed view of one mapped page and its topic boundary. */
template <PagePod T, PagePod P, uint32_t TAG = 0>
class SharedPage : public Topic
{
 public:
  /// @brief The contract instantiation's layout, shared by both cores.
  static constexpr PageLayout Layout() { return detail::MakeLayout<T, P, TAG>(); }

  SharedPage() = default;

  /// @brief Bind one mapped page.
  explicit SharedPage(void* addr) : core_(addr, Layout()) {}
  SharedPage(void* addr, Topic topic) : Topic(topic), core_(addr, Layout()) {}

  [[nodiscard]] bool Valid() const { return core_.Valid(); }
  [[nodiscard]] uint8_t* Data() const { return core_.Data(); }
  [[nodiscard]] PageMagicKind Check() const { return core_.Check(); }
  void Format() { core_.Format(); }
  void ClearHistory() { core_.ClearHistory(); }

  /// @brief Return whether the page is ready.
  [[nodiscard]] bool Ready() const { return Check() == PageMagicKind::FORMATTED; }

  /// @brief Return the telemetry writer view.
  [[nodiscard]] Telemetry<T> TelemetryWriter() const
  {
    return Telemetry<T>(Data(), Layout());
  }

  /// @brief Return the telemetry reader view.
  [[nodiscard]] Telemetry<T> TelemetryReader() const { return TelemetryWriter(); }

  /// @brief Return the region view.
  [[nodiscard]] Reference<P> Region() const { return Reference<P>(Data(), Layout()); }

  /// @brief Write one telemetry sample.
  uint32_t WriteSample(const T& sample) { return TelemetryWriter().Write(sample); }

  /// @brief Read the latest telemetry sample.
  [[nodiscard]] bool Latest(T* sample) const { return TelemetryReader().Latest(sample); }

 private:
  detail::PageCore core_;
};

}  // namespace LibXR
