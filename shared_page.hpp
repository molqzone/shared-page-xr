#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "libxr.hpp"

/**
 * @file shared_page.hpp
 * @brief Platform-neutral shared-page semantics over an externally supplied frame.
 *
 * 三层各有其主：**帧/几何是平台数据**（页多大、多少槽、什么偏移，由平台内存图与
 * ABI 定，例如 sophgo-sg200x-debian 的 `sg2002-ipc`），**载荷是应用数据**（遥测采样
 * 与参考区 payload 的字段），本库只拥有第三层——**同步语义**：keep-latest 遥测环、
 * 有界历史的 gap 规则、自洽区域快照，以及 `head`/`seq`/`write_state` 的
 * acquire-release 协议。
 *
 * 几何在**编译期**以 format traits 注入（`DocPageFormat` 只是文档定稿那份格式，不是
 * 默认真理），在**初始化期**由自描述页头逐项互验——页头照 `sg2002-ipc`
 * `layout_line` 的做法把每个几何数字写出来，`Check()` 逐项比对，任何漂移报
 * `MISMATCH`，绝不静默错读。
 *
 * Three layers, three owners: the frame geometry is platform data (how large the page
 * is, how many slots, which offsets -- dictated by the platform memory map and ABI,
 * e.g. `sg2002-ipc` in sophgo-sg200x-debian), the payloads are application data (the
 * telemetry sample and the region payload fields), and this library owns only the
 * semantics: the keep-latest telemetry ring, the bounded-history gap rule, coherent
 * region snapshots, and the acquire-release `head`/`seq`/`write_state` protocol.
 *
 * Geometry is injected at compile time as format traits and cross-verified at
 * initialization through a self-describing header. The header records every geometry
 * number the way `sg2002-ipc`'s `layout_line` does, and `Check()` compares them one
 * by one: any drift is reported as `MISMATCH`, never silently misread.
 */

namespace LibXR
{
/** @brief Frame format of one platform: the geometry the page contract plays on.
 *
 * 平台数据的载体：页多大、多少槽、帧 ABI 第几版。产品在自己的契约头里指向平台要求
 * 的那份（本仓库的 `DocPageFormat` 对应 `inter-core-protocol.md` 定稿；若平台要求
 * 别的几何，换 traits 即可，语义与 API 不动）。
 * The carrier of platform data: page size, slot count, frame ABI version. The product
 * points at whatever its platform mandates in its own contract header (`DocPageFormat`
 * here matches the `inter-core-protocol.md` final draft; different geometry means a
 * different traits type, with the semantics and API unchanged).
 */
struct DocPageFormat
{
  static constexpr uint32_t ABI_VERSION = 1;
  static constexpr size_t PAGE_SIZE = 4096;
  static constexpr uint32_t SLOT_COUNT = 64;
};

/// @brief Page header magic.
inline constexpr uint32_t PAGE_MAGIC = 0x31506461U;  // NOLINT

/** @brief Self-describing page header.
 *
 * 与 `sg2002_ipc` 的 `sg2002_rtos_shm_layout_line` 同一做法：几何不藏在常量或指纹
 * 里，而是写在页上，双端 `Check()` 逐项互验。
 * The same practice as `sg2002_ipc`'s `sg2002_rtos_shm_layout_line`: the geometry
 * lives on the page instead of hiding in constants or a digest, and both ends verify
 * it field by field in `Check()`.
 */
struct PageHeader
{
  uint32_t magic;
  uint32_t abi_version;
  uint32_t page_size;
  uint32_t slot_count;
  uint32_t sample_size;
  uint32_t payload_size;
  uint32_t region_offset;
  uint32_t tag;
};

static_assert(sizeof(PageHeader) == 32, "PageHeader layout pinned");
static_assert(offsetof(PageHeader, tag) == 28, "PageHeader layout pinned");

/// @brief In-page telemetry offset (header plus alignment padding).
inline constexpr size_t telemetry_offset() { return sizeof(PageHeader); }

/** @brief Result of validating a mapped page.
 *
 * `MISMATCH` 区别于 `FOREIGN`：魔术字正确但几何/载荷/标签对不上，说明双端用了不同
 * 版本的帧或 wire 结构（例如固件热替换后的新旧配对）。
 * `MISMATCH` differs from `FOREIGN`: the magic is right but the geometry, payloads or
 * tag do not match, meaning the two ends were built against a different frame or wire
 * structure (for example a stale pairing after a runtime firmware replacement).
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

/** @brief Telemetry ring and publication state for one sample type and slot count. */
template <PagePod T, uint32_t SLOTS>
struct TelemetryRing
{
  T ring[SLOTS];  ///< 定长历史，写者覆写最旧槽。Fixed history; the writer
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

/// @brief In-page reference offset for one format and payload type.
template <typename F, PagePod P>
inline constexpr size_t region_offset()
{
  return F::PAGE_SIZE - sizeof(Region<P>);
}

/** @brief Compile-time layout of one contract instantiation.
 *
 * 双端各自从同一份 format traits 与 wire 类型算出这份描述；页头记录它的每个数字，
 * 供 `Check()` 逐项互验。
 * Each end computes this description from the same format traits and wire types; the
 * header records every number of it for `Check()` to verify field by field.
 */
struct PageLayout
{
  uint32_t abi_version = 0;
  uint32_t page_size = 0;
  uint32_t slot_count = 0;
  uint32_t sample_size = 0;
  uint32_t payload_size = 0;
  uint32_t region_offset = 0;
  uint32_t tag = 0;
};

namespace detail
{
/** @brief Compute and validate the layout of `SharedPage<F, T, P, TAG>`. */
template <typename F, PagePod T, PagePod P, uint32_t TAG = 0>
constexpr PageLayout MakeLayout()
{
  using Ring = TelemetryRing<T, F::SLOT_COUNT>;

  static_assert(F::ABI_VERSION >= 1, "the frame must declare an ABI version");
  static_assert(F::PAGE_SIZE >= sizeof(PageHeader), "the page must hold its header");
  static_assert(F::PAGE_SIZE % sizeof(uint32_t) == 0,
                "the page must stay word-atomic for the in-page copies");
  static_assert(F::SLOT_COUNT >= 2, "the ring needs at least two slots");
  static_assert(sizeof(T) % sizeof(uint32_t) == 0,
                "the sample type must be a multiple of 4 bytes: ring copies are "
                "word-atomic");
  static_assert(sizeof(P) % sizeof(uint32_t) == 0,
                "the payload type must be a multiple of 4 bytes: region copies are "
                "word-atomic");
  static_assert(telemetry_offset() % sizeof(uint32_t) == 0 &&
                    (sizeof(T) * F::SLOT_COUNT) % sizeof(uint32_t) == 0,
                "every ring slot must start 4-byte aligned inside the page");
  static_assert(region_offset<F, P>() % sizeof(uint32_t) == 0,
                "the region must start 4-byte aligned inside the page");
  static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
                "the in-page counters must occupy exactly 4 bytes");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "the in-page counters must be lock-free");
  static_assert(offsetof(Ring, head) == sizeof(T) * F::SLOT_COUNT,
                "head must follow the ring without padding");
  static_assert(offsetof(Region<P>, write_state) == sizeof(P),
                "write_state must follow the payload without padding");
  static_assert(offsetof(Region<P>, seq) == sizeof(P) + sizeof(uint32_t),
                "seq must follow write_state");
  static_assert(telemetry_offset() + sizeof(Ring) <= region_offset<F, P>(),
                "the telemetry ring and the region must not overlap");

  return PageLayout{F::ABI_VERSION,
                    static_cast<uint32_t>(F::PAGE_SIZE),
                    F::SLOT_COUNT,
                    static_cast<uint32_t>(sizeof(T)),
                    static_cast<uint32_t>(sizeof(P)),
                    static_cast<uint32_t>(region_offset<F, P>()),
                    TAG};
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
  uint32_t slots_ = 0;
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
  void RecoverStaleClaims();

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
   * 单一入口、`seq` 必填形参：不提供 `Read(out, retries)` 重载，否则 `Read(out, 0)``
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

/** @brief Typed view of one mapped page and its topic boundary.
 *
 * `F` 是平台帧格式，`T`/`P` 是应用载荷，`TAG` 是应用侧 wire 版本标签。
 * `F` is the platform frame format, `T`/`P` the application payloads, and `TAG` the
 * application-side wire version tag.
 */
template <typename F, PagePod T, PagePod P, uint32_t TAG = 0>
class SharedPage : public Topic
{
 public:
  /// @brief The contract instantiation's layout, shared by both cores.
  static constexpr PageLayout Layout() { return detail::MakeLayout<F, T, P, TAG>(); }

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

  /** @brief Break write-state claims left behind by a dead peer writer.
   *
   * 写者在 claim 窗口内死亡（进程被杀/固件崩溃）会把 `write_state` 永久留在占用态，
   * 读者从此只拿到 `BUSY`。本操作只清两个 claim，不动 `head`/`seq`/数据：未发布完的
   * 槽位在 `head` 之外，读者看不到，所以破 claim 不会漏出撕裂数据，也不需要丢历史。
   * 调用者带时钟判断"持续 BUSY 超阈值 = 写者已死"（见 Linux 适配器的 Drain）；写者
   * 只是短暂持有 claim 时必须用正常路径，不要调本函数。
   *
   * A writer dying inside the claim window (killed process, crashed firmware) leaves
   * `write_state` claimed forever and readers get `BUSY` from then on. This only
   * breaks the two claims; `head`/`seq`/data are untouched: the in-flight slot lives
   * beyond `head` and is invisible to readers, so breaking the claim exposes no torn
   * data and loses no history. Callers own the clock and the "BUSY for longer than a
   * threshold means the writer is dead" verdict (see the Linux adapter's Drain);
   * while a live writer merely holds the claim, use the normal paths instead.
   */
  void RecoverStaleClaims() { core_.RecoverStaleClaims(); }

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
