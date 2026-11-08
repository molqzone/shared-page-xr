#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "crc.hpp"
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
 * 几何在**编译期**以 format traits 注入（本库不带任何 SoC 的默认几何），在**初始化期**
 * 由自描述页头逐项互验——页头照 `sg2002-ipc`
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
/** @brief 平台帧几何必须给出的三个数 / The three numbers a platform geometry provides.
 *
 * 本库**不带任何 SoC 的默认几何**：页多大、多少槽、帧 ABI 第几版都是平台数据，由消费方
 * 在自己的契约头里声明（本板是 `User/guidance_contract.hpp` 的 `Guidance::Format`，
 * 语义见 `docs/inter-core-protocol.md`）。换几何 = 换 traits，语义与 API 不动。
 * This library ships **no SoC's default geometry**: page size, slot count and frame ABI
 * version are platform data, declared by the consumer in its own contract header (on this
 * board `Guidance::Format` in `User/guidance_contract.hpp`, specified by
 * `docs/inter-core-protocol.md`).  A different geometry is a different traits type, with
 * the semantics and API unchanged.
 *
 * 这条约束写成 concept 而不是注释：漏一个成员时错在声明处，而不是在某个 `static_assert`
 * 的深处。
 * The constraint is a concept rather than a comment: a missing member fails at the
 * declaration, not deep inside some `static_assert`.
 */
template <typename F>
concept PageFormat = requires {
  F::ABI_VERSION;
  F::PAGE_SIZE;
  F::SLOT_COUNT;
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

/// @brief Cache-line size the page layout aligns to.
///
/// Every independently-maintained structure — the ring together with its
/// trailing `head`/`write_state`, and whatever the product places after it —
/// starts on its own cache line, so a whole-line clean/invalidate on one never
/// sweeps another's bytes.  64 matches the C906 / Cortex-A line (and
/// `LibXR::HW_CACHE_LINE_SIZE` on the 64-bit targets this transport runs on);
/// over-aligning is safe, under-aligning is not.  Promote to a format trait if a
/// platform ever needs a different line.
inline constexpr size_t PAGE_CACHE_LINE = 64;

/// @brief Round a byte count up to a multiple of `align` (a power of two).
inline constexpr size_t align_up(size_t value, size_t align)
{
  return (value + (align - 1)) & ~(align - 1);
}

/// @brief In-page ring offset (header rounded up to the cache line).
///
/// `PageHeader` is 32 bytes, not a cache-line multiple, so starting the ring at
/// `sizeof(PageHeader)` leaves the ring's `head`/`write_state` mid-line, sharing
/// a line with whatever the product places next (a name-service table, say) —
/// the same false-sharing class that forced the console ring onto its own page.
/// Rounding up puts the ring, and therefore its counters, on a line boundary.
inline constexpr size_t telemetry_offset()
{
  return align_up(sizeof(PageHeader), PAGE_CACHE_LINE);
}

/** @brief Result of validating a mapped page.
 *
 * `MISMATCH` 区别于 `FOREIGN`：魔术字正确但几何/载荷/标签对不上，说明双端用了不同
 * 版本的帧或 wire 结构（例如固件热替换后的新旧配对）。
 * `MISMATCH` differs from `FOREIGN`: the magic is right but the geometry, payloads or
 * tag do not match, meaning the two ends were built against a different frame or wire
 * structure (for example a stale pairing after a runtime firmware replacement).
 */
enum class PageMagicStatus : uint8_t
{
  FORMATTED = 0,
  UNFORMATTED = 1,
  FOREIGN = 2,
  MISMATCH = 3,
};

/** @brief Name of a page status, for logs and CLI output. */
[[nodiscard]] inline const char* MagicName(PageMagicStatus status)
{
  switch (status)
  {
    case PageMagicStatus::FORMATTED:
      return "FORMATTED";
    case PageMagicStatus::UNFORMATTED:
      return "UNFORMATTED";
    case PageMagicStatus::FOREIGN:
      return "FOREIGN";
    case PageMagicStatus::MISMATCH:
      return "MISMATCH";
  }
  return "UNKNOWN";
}

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
template <PageFormat F, PagePod P>
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


/** @brief 页上的一个通道条目 / One announced channel on the page.
 *
 * 名字是服务/数据的标识（与 `LibXR::Topic` 的键一致：名字的 CRC32），端点是它的投递
 * 口。条目不参与 `head`/`seq` 协议：整张表由 `gen` 一次通告，读侧要么看到一整代、要么
 * 看到上一代。
 * The name identifies a service or stream (the same key `LibXR::Topic` uses: the
 * CRC32 of the name) and the endpoint is where it is delivered.  Entries are not part
 * of the `head`/`seq` protocol: the table is announced as one generation, and a reader
 * sees a whole generation or the previous one.
 */
struct ChannelEntry
{
  uint32_t key;         ///< 名字的 CRC32（与 `LibXR::CRC32` 一致）。CRC32 of the name.
  uint32_t endpoint;    ///< 端点地址。Endpoint address.
  uint32_t flags;       ///< 见 `CHANNEL_FLAG_*`。See `CHANNEL_FLAG_*`.
  uint16_t name_len;    ///< 名字字节数（<= sizeof(name)）。Name length in bytes.
  char name[28];        ///< 名字（不以 0 结尾也合法）。Name; not necessarily terminated.
} __attribute__((packed));

static_assert(sizeof(ChannelEntry) == 42, "a channel entry is 42 bytes on the wire");

/// @brief 通道存在（对应 rpmsg `NS_CREATE`）。The channel exists (rpmsg NS_CREATE).
inline constexpr uint32_t CHANNEL_FLAG_PRESENT = 1u;

/// @brief 通道表魔数 "NS01"（沿用 rpmsg NS 区时代的记号）。Channel-table magic.
inline constexpr uint32_t CHANNEL_TABLE_MAGIC = 0x3130534EU;

/** @brief 页上的通道表：一代通告 + `CAPACITY` 个条目。
 *         The page's channel table: one announced generation plus `CAPACITY` entries.
 *
 * 表头自描述（魔数、有效条目数、代次），所以“还没通告”与“通告了空表”可区分：前者魔数
 * 不匹配，后者 `count == 0`。
 * The header describes itself (magic, live count, generation), so "never announced" and
 * "announced as empty" stay distinguishable: the former fails the magic, the latter has
 * `count == 0`.
 */
template <uint32_t CAPACITY>
struct ChannelTable
{
  uint32_t magic;   ///< CHANNEL_TABLE_MAGIC。
  uint32_t count;   ///< 有效条目数。Live entries.
  uint32_t gen;     ///< 每次通告递增。Bumped on every announcement.
  uint32_t reserved;
  ChannelEntry entries[CAPACITY];
};

/** @brief 一条待通告的通道：名字、端点、flags。
 *         One channel to announce: name, endpoint, presence flags. */
struct ChannelAnnouncement
{
  const char* name;   ///< 通道名（不含核名前缀）。Channel name, no core prefix.
  uint32_t endpoint;  ///< 端点地址。Endpoint address.
  uint32_t flags;     ///< `CHANNEL_FLAG_PRESENT` 等。Presence flags.
};

/** @brief Compute and validate the layout of `SharedPage<F, T, P, TAG>`. */
template <PageFormat F, PagePod T, PagePod P, uint32_t TAG = 0>
constexpr PageLayout make_page_layout()
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
  static_assert((telemetry_offset() + sizeof(T) * F::SLOT_COUNT) % PAGE_CACHE_LINE == 0,
                "the ring must end on a cache line so head/write_state start their "
                "own line, isolated from the product area that follows");

  return PageLayout{F::ABI_VERSION,
                    static_cast<uint32_t>(F::PAGE_SIZE),
                    F::SLOT_COUNT,
                    static_cast<uint32_t>(sizeof(T)),
                    static_cast<uint32_t>(sizeof(P)),
                    static_cast<uint32_t>(region_offset<F, P>()),
                    TAG};
}


/** @brief 通道表在页内的偏移：环的计数器之后的那一条缓存行。
 *         Offset of the channel table: the cache line after the ring's counters.
 *
 * 只依赖布局描述，所以两端从同一份 `PageLayout` 得到同一个偏移；表从缓存行开始，与
 * `head`/`write_state` 不同行，一次整表维护不会顺带推回环的计数。
 * Derived from the layout description alone, so both ends get the same offset from the
 * same `PageLayout`; the table starts on its own cache line, so maintaining it never
 * sweeps the ring's counters.
 */
inline constexpr size_t channel_offset(const PageLayout& layout)
{
  return align_up(telemetry_offset() +
                      static_cast<size_t>(layout.slot_count) * layout.sample_size +
                      2 * sizeof(uint32_t),
                  PAGE_CACHE_LINE);
}

/// @brief 这张容量的通道表放得进页的尾部区之前吗？
///        Does a table of this capacity fit before the page's trailing region?
template <uint32_t CAPACITY>
inline constexpr bool channel_table_fits(const PageLayout& layout)
{
  return channel_offset(layout) + sizeof(ChannelTable<CAPACITY>) <= layout.region_offset;
}

/** @brief 一次访问想让谁看见 / Who one access must become visible to.
 *
 * 意图是调用点唯一要回答的问题；“做什么维护”由这个域绑进来的 provider 决定。
 * 这正是 page 与外设驱动（DMA 的 clean/clean+invalidate/invalidate）共用的同一组
 * 答案，只是调用点问法不同。
 * Intent is the only question the call site answers; the provider turns it into
 * maintenance.  These are the same three answers the DMA driver already exposes
 * (`CleanForDevice` / `PrepareForDeviceWrite` / `InvalidateForCpu`), asked from
 * the accessor's side instead of the device's.
 */
enum class AccessIntent : uint8_t
{
  /** 我要读对端写的字节：进入时丢弃本地过期行。Read peer bytes: refresh on enter. */
  READ,
  /** 我写完整片范围：离开时把脏行推回。Write the whole range: publish on exit. */
  WRITE,
  /** 读改写：两边的副本都不可信，进入时同步、离开时推回。Read-modify-write:
   *  neither copy is authoritative, so sync on enter and publish on exit. */
  EXCHANGE,
  /** 我写完整片范围，且写完后本地副本不可信：离开时推回并作废（clean+invalidate）。
   *  Write the whole range and distrust the local copy afterwards: push it back and
   *  drop it on exit (clean+invalidate).
   *
   *  与 `WRITE` 的差别只在退出动作：`WRITE` 只 clean，`WRITE_SYNC` clean+invalidate。
   *  需要它的是"写完马上就要被读回"的场合——冷页格式化跑在 bring-up 最早的位置，
   *  缓存状态还没到稳态（C906 的维护指令与 `MHCR.DE` 的时序坑见
   *  `driver/shared_page/sg200x_thead_coherence.hpp`），只 clean 不足以证明字节已经
   *  到了 DRAM。
   *  It differs from `WRITE` only in the exit action: `WRITE` cleans, `WRITE_SYNC`
   *  cleans and invalidates.  It is for "written, then read back right away" --
   *  cold-page formatting runs at the earliest point of bring-up, where the cache is
   *  not yet in its steady state (see the provider header for the C906 maintenance /
   *  `MHCR.DE` timing), and a clean alone does not prove the bytes reached DRAM. */
  WRITE_SYNC,
};

/** @brief 一个能为一致性域提供三个动作的类型 / A type that can serve a coherency domain.
 *
 * 名字说的是**角色**，不是芯片：`THeadDcacheCoherence` 是这个角色的一个实现，
 * `NoCoherence` 是“对端不存在”那一域的答案。“这段字节属于哪个域”由绑定点回答——
 * 页别名的类型实参就是这一端的答案；“什么时候维护、维护哪些字节”由 `AccessWindow`
 * 按访问意图问。所以一致性从来不是某一颗核的属性：单核自己的缓存谈不上“一致不一致”。
 *
 * 它**不含地址，也不含长度**：那是 `AccessWindow`（一次访问）与 `Reference`（一段
 * 字节的视图）的事。所以它不是一个 `Region`，只是 Region 模型里“这一域的答案”那一半。
 *
 * 类型是**编译期**的：三个 const 动作，`AccessWindow` 静态调用它们，`NoCoherence`
 * 被完全内联掉，真实实现也没有间接调用（`THeadDcacheCoherence` 直接调 arch 原语）。
 *
 * 抽象的是**这次访问想让谁看见**，不是 cache 本身：对端是“共 RAM 不共 cache 的核”
 * 时动作是 clean/invalidate，对端是绕过 cache 的 DMA 设备时也是同一组动作，对端不
 * 存在（NC 映射、单地址空间）时三个动作都是空。
 *
 * 动作的实现还必须同时是编译器屏障（跨 TU 调用或带 `memory` clobber 的内联汇编），
 * 否则编译器和硬件都可能把数据访问挪到维护之外。类型只强制三个动作存在，屏障这一条
 * 由实现保证（`driver/shared_page/sg200x_thead_coherence.hpp` 走带 `sync.s` 的跨 TU 调用）。
 *
 * Role, not chip: `THeadDcacheCoherence` is one implementation of this role and
 * `NoCoherence` is the answer for a domain with no peer.  *Which* domain these bytes
 * belong to is decided at the binding site -- the page alias's type argument is this
 * side's answer -- and *when* and *over which bytes* to maintain is asked per access
 * through `AccessWindow`.  That is why coherency is never a property of one core: a
 * single core's own cache cannot be "coherent" or "incoherent".
 *
 * It carries **neither address nor length**: those belong to `AccessWindow` (one access)
 * and `Reference` (a view over bytes).  So it is not a `Region`; it is the "answer for
 * this domain" half of that model.
 *
 * The type is **compile-time**: three const actions that `AccessWindow` calls
 * statically, so `NoCoherence` is inlined away entirely and a real implementation costs
 * no indirect call (`THeadDcacheCoherence` reaches the arch primitives).
 *
 * The actions must also be compiler barriers (a cross-TU call, or inline asm with a
 * `memory` clobber), or the compiler and the hardware may move data accesses across
 * them.  The type can force the three actions to exist; the barrier is the
 * implementation's job.
 */
template <typename Provider>
concept Coherent = requires(const Provider& provider, void* address,
                            const void* read_address, size_t bytes) {
  provider.Publish(address, bytes);
  provider.Refresh(read_address, bytes);
  provider.Sync(address, bytes);
};

/** @brief 对端不存在的域（NC 映射 / 单地址空间）：三个动作都是空。
 *         A domain with no peer (NC mapping, single address space): all three actions
 *         are empty. */
struct NoCoherence
{
  void Publish(void*, size_t) const {}
  void Refresh(const void*, size_t) const {}
  void Sync(void*, size_t) const {}
};

/** @brief 函数指针桥：平台只给三个自由函数时 / A bridge: the platform hands over three
 *         free functions.
 *
 * 这是嵌入式里最常见的形态（DMA 驱动、arch 缓存原语），所以留着这座桥。它每个动作多
 * 一次间接调用；要零间接就用真正的实现（`THeadDcacheCoherence`）。
 *
 * Kept because "the platform hands over three functions" is the common embedded shape
 * (DMA drivers, arch cache primitives).  It costs one indirect call per action; a real
 * implementation costs none (`THeadDcacheCoherence`).
 */
struct FunctionPointerCoherence
{
  using PublishFn = void (*)(void* address, size_t bytes);
  using RefreshFn = void (*)(const void* address, size_t bytes);
  using SyncFn = void (*)(void* address, size_t bytes);

  constexpr FunctionPointerCoherence(PublishFn publish, RefreshFn refresh, SyncFn sync)
      : publish_(publish), refresh_(refresh), sync_(sync)
  {
  }

  /** @brief 我的写要让对端看见：clean。Make my writes visible: clean. */
  void Publish(void* address, size_t bytes) const { publish_(address, bytes); }

  /** @brief 我要读对端的写：invalidate。Discard my stale view: invalidate. */
  void Refresh(const void* address, size_t bytes) const { refresh_(address, bytes); }

  /** @brief 两边都不可信：clean+invalidate。Neither copy is authoritative. */
  void Sync(void* address, size_t bytes) const { sync_(address, bytes); }

 private:
  PublishFn publish_;
  RefreshFn refresh_;
  SyncFn sync_;
};

/** @brief 一段受一致性维护的访问窗口 / A maintenance window over one byte range.
 *
 * 进入时按意图做“读前”的动作，离开时做“写后”的动作；调用点因此不可能只配一半。
 * 只声明、不默认构造：窗口必须有策略与范围。策略是编译期类型，动作内联。
 *
 * Applies the "before" action on entry and the "after" action on exit, by intent, so a
 * call site cannot apply half of a pair.  Deliberately not default constructible: a
 * window always has a provider and a range.  The provider is a compile-time type, so the
 * actions inline.
 */
template <Coherent Provider>
class AccessWindow
{
 public:
  AccessWindow(const Provider& provider, const void* address, size_t bytes,
               AccessIntent intent)
      : provider_(provider), address_(const_cast<void*>(address)), bytes_(bytes), intent_(intent)
  {
    if (bytes_ == 0 || address_ == nullptr)
    {
      active_ = false;
      return;
    }
    if (intent_ == AccessIntent::READ)
    {
      provider_.Refresh(address_, bytes_);
    }
    else if (intent_ == AccessIntent::EXCHANGE)
    {
      provider_.Sync(address_, bytes_);
    }
  }

  ~AccessWindow()
  {
    if (!active_)
    {
      return;
    }
    if (intent_ == AccessIntent::WRITE_SYNC)
    {
      provider_.Sync(address_, bytes_);
    }
    else if (intent_ == AccessIntent::WRITE || intent_ == AccessIntent::EXCHANGE)
    {
      provider_.Publish(address_, bytes_);
    }
  }

  AccessWindow(const AccessWindow&) = delete;
  AccessWindow& operator=(const AccessWindow&) = delete;
  AccessWindow(AccessWindow&&) = delete;
  AccessWindow& operator=(AccessWindow&&) = delete;

 private:
  const Provider& provider_;
  void* address_;
  size_t bytes_;
  AccessIntent intent_;
  bool active_ = true;
};


/** @brief 页上通道表的 typed 视图 / Typed view of the page's channel table.
 *
 * 通告与发现都是列表动作，不是热路径：`Publish` 把一整代写上去（先在窗口外校验名字，
 * 再在 `EXCHANGE` 窗口里整表替换），`Find` 在 `READ` 窗口里按名字查一条。维护由视图
 * 带着页的策略做——调用方不需要认识缓存操作。
 * Announcing and discovering are list operations, not a hot path: `Publish` writes one
 * whole generation (names validated before the window, table replaced inside an
 * `EXCHANGE` window) and `Find` looks one up by name inside a `READ` window.  The view
 * carries the page's provider, so a caller never handles cache maintenance itself.
 *
 * `CAPACITY` 是这张表在页布局里占的条目数；放不放得下由 `channel_table_fits()` 在编译期
 * 回答，构造时不再猜。
 * `CAPACITY` is how many entries this table reserves in the page layout;
 * `channel_table_fits()` answers at compile time whether that fits, so construction
 * does not guess.
 */
template <uint32_t CAPACITY, Coherent Provider>
class NameService
{
 public:
  NameService() = default;

  /// @brief 绑定到一页（`addr` 是页基址，偏移由布局决定）。Bind to one page.
  NameService(void* addr, const PageLayout& layout, const Provider& provider)
      : table_(addr == nullptr
                   ? nullptr
                   : reinterpret_cast<ChannelTable<CAPACITY>*>(
                         static_cast<uint8_t*>(addr) + channel_offset(layout))),
        provider_(provider)
  {
  }

  /// @brief 这页上有没有一代有效的通告。Whether the table carries a valid generation.
  [[nodiscard]] bool Valid() const
  {
    if (table_ == nullptr)
    {
      return false;
    }
    AccessWindow window(provider_, table_, sizeof(ChannelTable<CAPACITY>),
                        AccessIntent::READ);
    return table_->magic == CHANNEL_TABLE_MAGIC && table_->count <= CAPACITY;
  }

  /// @brief 当前通告的代次（没通告过返回 0）。Generation of the current announcement.
  [[nodiscard]] uint32_t Generation() const
  {
    if (table_ == nullptr)
    {
      return 0u;
    }
    AccessWindow window(provider_, table_, sizeof(ChannelTable<CAPACITY>),
                        AccessIntent::READ);
    return table_->magic == CHANNEL_TABLE_MAGIC ? table_->gen : 0u;
  }

  /** @brief 通告这一代通道 / Announce this generation of channels.
   *
   * 只登记现存服务，不预占假想通道；代次自增，读侧据此识别新通告。名字先校验（空名或
   * 放不进条目的名字直接拒绝），再整表替换——读者看到的永远是完整的一代。
   * Only services that exist today, never reserved names; the generation is bumped so a
   * reader can tell a new announcement apart.  Names are validated first (empty or
   * oversized names are refused), then the whole table is replaced, so a reader always
   * sees a complete generation.
   */
  bool Publish(const ChannelAnnouncement* channels, size_t count)
  {
    if (table_ == nullptr || channels == nullptr || count == 0 || count > CAPACITY)
    {
      return false;
    }
    for (size_t index = 0; index < count; ++index)
    {
      const size_t length = std::strlen(channels[index].name);
      if (length == 0 || length > sizeof(ChannelEntry::name))
      {
        return false;
      }
    }

    AccessWindow window(provider_, table_, sizeof(ChannelTable<CAPACITY>),
                        AccessIntent::EXCHANGE);
    const uint32_t generation = table_->gen + 1u;
    table_->magic = CHANNEL_TABLE_MAGIC;
    table_->count = static_cast<uint32_t>(count);
    table_->gen = generation;
    table_->reserved = 0;
    for (size_t index = 0; index < count; ++index)
    {
      const ChannelAnnouncement& channel = channels[index];
      ChannelEntry& entry = table_->entries[index];
      const size_t length = std::strlen(channel.name);
      entry.key = CRC32::Calculate(channel.name, length);
      entry.endpoint = channel.endpoint;
      entry.flags = channel.flags;
      entry.name_len = static_cast<uint16_t>(length);
      std::memset(entry.name, 0, sizeof(entry.name));
      std::memcpy(entry.name, channel.name, length);
    }
    for (size_t index = count; index < CAPACITY; ++index)
    {
      table_->entries[index] = ChannelEntry{};
    }
    return true;
  }

  /** @brief 按名字查一条通道 / Look one channel up by name.
   *
   * 名字比较的是字节与长度，键（CRC32）只是给按 key 索引的读者用的；碰撞不会让一个
   * 名字匹配到另一个。
   * The lookup compares bytes and length; the key (CRC32) exists for readers that index
   * by key, and a collision cannot make one name match another.
   */
  bool Find(const char* name, ChannelEntry* out) const
  {
    if (table_ == nullptr || name == nullptr)
    {
      return false;
    }
    const size_t length = std::strlen(name);
    if (length == 0 || length > sizeof(ChannelEntry::name))
    {
      return false;
    }

    AccessWindow window(provider_, table_, sizeof(ChannelTable<CAPACITY>),
                        AccessIntent::READ);
    if (table_->magic != CHANNEL_TABLE_MAGIC || table_->count > CAPACITY)
    {
      return false;
    }
    for (uint32_t index = 0; index < table_->count; ++index)
    {
      const ChannelEntry& entry = table_->entries[index];
      if (entry.name_len != length || std::memcmp(entry.name, name, length) != 0)
      {
        continue;
      }
      if (out != nullptr)
      {
        *out = entry;
      }
      return true;
    }
    return false;
  }

 private:
  ChannelTable<CAPACITY>* table_ = nullptr;
  [[no_unique_address]] Provider provider_{};
};

/** @brief Type-erased view of the telemetry ring. Protocol engine, see the .cpp. */
template <Coherent Provider>
class TelemetryCore
{
 public:
  TelemetryCore() = default;
  TelemetryCore(void* addr, const PageLayout& layout,
                const Provider& provider);

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
  [[no_unique_address]] Provider provider_{};
};

/** @brief Type-erased view of the region. Protocol engine, see the .cpp. */
template <Coherent Provider>
class ReferenceCore
{
 public:
  ReferenceCore() = default;
  ReferenceCore(void* addr, const PageLayout& layout,
                const Provider& provider);

  [[nodiscard]] ErrorCode Read(void* out, uint32_t* seq, uint32_t retries) const;
  uint32_t Write(const void* payload);
  [[nodiscard]] uint32_t Seq() const;
  [[nodiscard]] const void* Raw() const;

 private:
  uint8_t* payload_ = nullptr;
  std::atomic<uint32_t>* state_ = nullptr;
  std::atomic<uint32_t>* seq_ = nullptr;
  uint32_t payload_size_ = 0;
  [[no_unique_address]] Provider provider_{};
};

/** @brief Type-erased page header operations. Protocol engine, see the .cpp. */
template <Coherent Provider>
class PageCore
{
 public:
  PageCore() = default;
  PageCore(void* addr, const PageLayout& layout,
           const Provider& provider);

  [[nodiscard]] bool Valid() const;
  [[nodiscard]] uint8_t* Data() const;
  [[nodiscard]] PageMagicStatus Check() const;

  /** @brief Bring the page up: build a cold page, refuse anything else.
   *
   *  冷页（全零、没人认领过）由这一侧建立；已建立的页直接复用；几何漂移
   *  （`MISMATCH`）与外来页（`FOREIGN`）**一律不格式化**，原样返回让调用方
   *  当致命错误——自动格式化会把 `TAG`/几何漂移这个唯一的探针抹掉。
   *
   *  A cold page (all zeroes, never claimed) is built by whichever side binds
   *  first; an already-formatted page is reused; a geometry drift (`MISMATCH`)
   *  or a foreign page (`FOREIGN`) is *never* formatted -- the status is
   *  returned for the caller to treat as fatal, because auto-formatting would
   *  erase the only probe for `TAG`/geometry drift.
   *
   *  @return `FORMATTED` when the page is usable, otherwise the reason it is not.
   */
  [[nodiscard]] PageMagicStatus Initialize();
  void Format();
  void ClearHistory();
  void RecoverStaleClaims();

  /// @brief The provider this page was bound with. The views inherit it.
  [[nodiscard]] const Provider& Coherence() const { return provider_; }

 private:
  uint8_t* page_ = nullptr;
  PageLayout layout_ = {};
  [[no_unique_address]] Provider provider_{};
};

/** @brief Typed view of the telemetry ring for one sample type. */
template <PagePod T, Coherent Provider>
class Telemetry
{
 public:
  Telemetry() = default;

  /// @brief Bind to a page.
  Telemetry(void* addr, const PageLayout& layout,
            const Provider& provider)
      : core_(addr, layout, provider)
  {
  }

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

 private:
  TelemetryCore<Provider> core_;
};

/** @brief Typed view of the region for one payload type. */
template <PagePod P, Coherent Provider>
class Reference
{
 public:
  Reference() = default;

  /// @brief Bind to a page.
  Reference(void* addr, const PageLayout& layout,
            const Provider& provider)
      : core_(addr, layout, provider)
  {
  }

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
  ReferenceCore<Provider> core_;
};

/** @brief Typed view of one mapped page and its topic boundary.
 *
 * `F` 是平台帧格式，`T`/`P` 是应用载荷，`TAG` 是应用侧 wire 版本标签。
 * `F` is the platform frame format, `T`/`P` the application payloads, and `TAG` the
 * application-side wire version tag.
 */
template <PageFormat F, PagePod T, PagePod P, uint32_t TAG = 0,
          Coherent Provider = FunctionPointerCoherence>
class SharedPage : public Topic
{
 public:
  /// @brief The contract instantiation's layout, shared by both cores.
  static constexpr PageLayout Layout() { return make_page_layout<F, T, P, TAG>(); }

  SharedPage() = default;

  /// @brief Bind one mapped page and state its coherency domain.
  ///
  /// 这个 provider 是**模板参数**：页必须回答“这段字节和谁共享、谁负责让它可见”，而
  /// 回答发生在编译期——需要维护式访问的平台给自己那侧的 `Page` 别名填
  /// `THeadDcacheCoherence`，对端不存在（NC 映射 / 单地址空间）填 `NoCoherence`。
  /// 默认值是函数指针桥，只为兼容“平台只给三个函数”的绑定点；产品契约不给默认。
  /// The provider is a **template argument**: a page must answer "who shares these bytes
  /// and who makes them visible", and it answers at compile time -- a platform with
  /// maintenance-mode access fills its own `Page` alias with `THeadDcacheCoherence`, a domain
  /// with no peer (NC mapping, single address space) with `NoCoherence`.  The default
  /// is the function-pointer provider, only for bindings where the platform hands over
  /// three functions; the product contract does not use the default.
  SharedPage(void* addr, const Provider& provider)
      : core_(addr, Layout(), provider)
  {
  }
  SharedPage(void* addr, Topic topic, const Provider& provider)
      : Topic(topic), core_(addr, Layout(), provider)
  {
  }

  [[nodiscard]] bool Valid() const { return core_.Valid(); }
  [[nodiscard]] uint8_t* Data() const { return core_.Data(); }

  /// @brief The coherency provider this page is bound to; a view created from this page
  ///        inherits it instead of silently dropping it.
  [[nodiscard]] const Provider& Coherence() const { return core_.Coherence(); }
  [[nodiscard]] PageMagicStatus Check() const { return core_.Check(); }

  /** @brief Bring the page up: build a cold page, refuse a drifted one.
   *
   *  发布方在开始写之前调一次。冷页（全零）由本侧建立；已建立的页复用；
   *  几何漂移（`MISMATCH`）与外来页（`FOREIGN`）原样返回，**不**自动格式化
   *  ——那正是 `TAG`/几何漂移唯一的探针。返回值不是 `FORMATTED` 就必须当
   *  致命错误处理：静默跳过发布曾经让整条遥测链停掉几天而无人发现。
   *
   *  A publisher calls this once before writing. A cold page is built by this
   *  side; a formatted page is reused; a drift (`MISMATCH`) or foreign page
   *  (`FOREIGN`) is returned as-is and *not* auto-formatted -- that is the only
   *  probe for `TAG`/geometry drift. Anything but `FORMATTED` must be treated
   *  as fatal: silently skipping the publish once stopped the whole telemetry
   *  path for days without anyone noticing.
   */
  [[nodiscard]] PageMagicStatus Initialize() { return core_.Initialize(); }
  void Format() { core_.Format(); }
  void ClearHistory() { core_.ClearHistory(); }

  /// @brief Return whether the page is ready.
  [[nodiscard]] bool Ready() const { return Check() == PageMagicStatus::FORMATTED; }

  /** @brief Break write-state claims left behind by a dead peer writer.
   *
   * 写者在 claim 窗口内死亡（进程被杀/固件崩溃）会把 `write_state` 永久留在占用态，
   * 读者从此只拿到 `BUSY`。本操作只清两个 claim，不动 `head`/`seq`/数据：未发布完的
   * 槽位在 `head` 之外，读者看不到，所以破 claim 不会漏出撕裂数据，也不需要丢历史。
   * 调用者带时钟判断"持续 BUSY 超阈值 = 写者已死"（板端工具与固件各自从自己的健康检查判断）；写者
   * 只是短暂持有 claim 时必须用正常路径，不要调本函数。
   *
   * A writer dying inside the claim window (killed process, crashed firmware) leaves
   * `write_state` claimed forever and readers get `BUSY` from then on. This only
   * breaks the two claims; `head`/`seq`/data are untouched: the in-flight slot lives
   * beyond `head` and is invisible to readers, so breaking the claim exposes no torn
   * data and loses no history. Callers own the clock and the "BUSY for longer than a
   * threshold means the writer is dead" verdict (the board tools and the firmware each decide it);
   * while a live writer merely holds the claim, use the normal paths instead.
   */
  void RecoverStaleClaims() { core_.RecoverStaleClaims(); }

  /// @brief Return the telemetry writer view.
  [[nodiscard]] Telemetry<T, Provider> TelemetryWriter() const
  {
    return Telemetry<T, Provider>(Data(), Layout(), core_.Coherence());
  }

  /// @brief Return the telemetry reader view.
  [[nodiscard]] Telemetry<T, Provider> TelemetryReader() const
  {
    return TelemetryWriter();
  }

  /// @brief Return the region view.
  [[nodiscard]] Reference<P, Provider> Region() const
  {
    return Reference<P, Provider>(Data(), Layout(), core_.Coherence());
  }

  /// @brief Return the name-service view for a table of `CAPACITY` entries.
  ///        返回 `CAPACITY` 个条目的通道表视图；能不能放下由 `channel_table_fits()` 在
  ///        编译期回答。
  template <uint32_t CAPACITY>
  [[nodiscard]] NameService<CAPACITY, Provider> Channels() const
  {
    return NameService<CAPACITY, Provider>(Data(), Layout(), core_.Coherence());
  }

  /// @brief Write one telemetry sample.
  uint32_t WriteSample(const T& sample) { return TelemetryWriter().Write(sample); }

  /// @brief Read the latest telemetry sample.
  [[nodiscard]] bool Latest(T* sample) const { return TelemetryReader().Latest(sample); }

 private:
  PageCore<Provider> core_;  ///< Owns the provider; the typed views inherit it from here.
};

}  // namespace LibXR
