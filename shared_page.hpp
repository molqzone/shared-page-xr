#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "libxr_def.hpp"

/**
 * @file shared_page.hpp
 * @brief SG2002 大小核共享页契约（平台中性，C606 直接用）。
 *        SG2002 inter-core shared-page contract, platform neutral and used
 *        directly by C606.
 *
 * 页是传输，Topic 是模块边界。本文件只有 POD 契约与发布索引原语，不含 Topic；Linux
 * 侧的适配在 `linux_shared_page.hpp`。契约定稿见
 * `bsp-guidance-vision/docs/inter-core-protocol.md`。
 * A page is the transport and a topic is the module boundary. This file holds only
 * the POD contract and the publish-index primitives, with no topic; the Linux-side
 * adapter is `linux_shared_page.hpp`. The settled contract is documented in
 * `bsp-guidance-vision/docs/inter-core-protocol.md`.
 */

namespace LibXR
{
/**
 * @brief 一页的大小：4 KiB，物理邻接分配的最小对齐单位。
 *        Size of one page: 4 KiB, the minimum alignment unit of physically
 *        contiguous allocation.
 */
inline constexpr size_t PAGE_SIZE = 4096;

/// @brief 页头部魔术字（`"SharedP1"`，小端字节序）。Page header magic.
inline constexpr uint32_t PAGE_MAGIC = 0x31506461U;  // NOLINT

/// @brief 访问单元（H.264 access unit）槽的魔术字（`"MailBox1"`）。Mailbox slot magic.
inline constexpr uint32_t ACCESS_UNIT_MAGIC = 0x31786F42U;  // NOLINT

/// @brief 遥测 ring 的槽位数。Slot count of the telemetry ring.
inline constexpr uint32_t TELEMETRY_SLOTS = 64;

/// @brief 单帧访问单元的最大字节数。Maximum bytes of one access-unit frame.
inline constexpr uint32_t MAILBOX_BYTES = 512 * 1024;

/// @brief 访问单元编码格式：未知。Access-unit encoding format: unknown.
inline constexpr uint32_t ACCESS_UNIT_FORMAT_UNKNOWN = 0;

/// @brief 访问单元编码格式：H.264 Annex-B。Access-unit encoding format: H.264 Annex-B.
inline constexpr uint32_t ACCESS_UNIT_FORMAT_H264_ANNEX_B = 1;

/**
 * @struct Sample
 * @brief 遥测记录：一次控制环采样。Telemetry record: one control-loop sample.
 *
 * 单位是硬件原生宽度：IMU 为原始 LSB、舵机为硬件命令字、tick 为 rdtime 计数。换算
 * 只发生在本就要计算的一端（Linux 侧几何解算），满量程刻度属 action 仓库的 IMU
 * 驱动配置。
 * Units are hardware native: raw LSB for the IMU, hardware command words for the
 * servos and rdtime counts for the tick. Conversion happens only where it is
 * needed anyway (the Linux-side geometry solver) and the full-scale factors belong
 * to the action repo's IMU driver configuration.
 *
 * 舵机通道数取自契约定稿的 4，`static_assert` 钉住数组长度与由此得到的布局。
 * The servo channel count is 4 as settled, and the static_asserts pin the array
 * length and the layout that follows from it.
 */
struct Sample
{
  uint64_t ticks;  ///< C606 tick（rdtime，25MHz 域）。C606 tick (rdtime, 25 MHz domain).
  int16_t accel[3];          ///< 加计原始 LSB。Raw accel LSB.
  int16_t gyro[3];           ///< 陀螺原始 LSB。Raw gyro LSB.
  uint16_t servo_target[4];  ///< 舵机硬件命令字（C606 解算输出）。Servo hardware command
                             ///< words, solved on the C606 side.
  uint32_t pad;              ///< 填充到 8B 对齐。Padding to 8-byte alignment.
};

static_assert(sizeof(Sample) == 32, "Sample must be 32B with 4 servo channels");
static_assert(offsetof(Sample, servo_target) == 20, "Sample::servo_target offset pinned");
static_assert(offsetof(Sample, pad) == 28, "Sample::pad offset pinned");

/**
 * @struct TelemetryRing
 * @brief 遥测区：ring[64] + 发布索引。Telemetry region: ring[64] plus publish index.
 *
 * `head` 是已发布条数（单调，不回绕），物理槽位为 `head % 64`；最新采样是
 * `ring[(head-1) % 64]`（`head == 0` 时无数据），区间为半开区间 `(last_seen, head]`。
 * `head` counts published samples (monotonic, never wraps) and the physical slot is
 * `head % 64`; the latest sample is `ring[(head-1) % 64]` (none when `head == 0`) and
 * a range is the half-open interval `(last_seen, head]`.
 */
struct TelemetryRing
{
  Sample ring[TELEMETRY_SLOTS];  ///< 定长历史，写者覆写最旧槽。Fixed history; the writer
                                 ///< overwrites the oldest slot.
  std::atomic<uint32_t> head;  ///< 已发布条数（release store）。Published count (release
                               ///< store).
  uint32_t reserved;           ///< 对齐与后续扩展留白。Alignment and future use.
};

static_assert(sizeof(TelemetryRing) == sizeof(Sample) * TELEMETRY_SLOTS + 8,
              "TelemetryRing layout pinned");

/**
 * @enum PageMagicKind
 * @brief 页校验结果。Result of validating a mapped page.
 */
enum class PageMagicKind : uint8_t
{
  FORMATTED = 0,    ///< 魔术字正确，页已格式化为该契约。Magic matches; page is formatted.
  UNFORMATTED = 1,  ///< 全零：刚上电、未格式化，或映射错地址。All zero: cold page or
                    ///< a wrong mapping.
  FOREIGN = 2,  ///< 有数据但不是本页：地址映射错误或双端契约不一致。Has data but not this
                ///< page: wrong mapping or a contract mismatch between the two cores.
};

/**
 * @enum RingScan
 * @brief `Telemetry::Since()` 的结果。Result of `Telemetry::Since()`.
 */
enum class RingScan : uint8_t
{
  DATA = 0,  ///< `samples` 有效。`samples` is valid.
  IDLE = 1,  ///< 无新数据（读者已追平写者）。No new data; the reader is caught up.
  GAP = 2,   ///< 整段丢弃（有界历史的固有竞态）。Whole range dropped (inherent race of
             ///< a bounded history).
};

/**
 * @enum RegionScan
 * @brief `Reference::Read()` 的结果。Result of `Reference::Read()`.
 */
enum class RegionScan : uint8_t
{
  CURRENT = 0,  ///< `region` 是自洽快照。`region` is a coherent snapshot.
  BUSY = 1,  ///< 重试次数用尽，快照可能撕裂。Retries exhausted; the snapshot may be torn.
};

/**
 * @brief 参考/命令区 payload（`seq` 由容器维护）。
 *        Reference/command region payload (`seq` is owned by the container).
 */
struct RegionPayload
{
  uint8_t found;      ///< 0 = 本帧无目标。0 = no target in this frame.
  uint8_t pad[3];     ///< 对齐留白。Alignment padding.
  float aim_x;        ///< 检测框中心 x，归一化 ±1 = 相机视场边缘。Detection-box centre
                      ///< x, normalised; ±1 is the camera field-of-view edge.
  float aim_y;        ///< 检测框中心 y（同上）。Detection-box centre y (same as above).
  uint8_t cmd;        ///< CMD_TRIGGER=1 / CMD_PARAM=2。Command code.
  uint8_t pad2[3];    ///< 对齐留白。Alignment padding.
  uint16_t param_id;  ///< CMD_PARAM 的 C606 侧参数表索引。Parameter-table index on the
                      ///< C606 side for CMD_PARAM.
  float value;  ///< f32；≤2^24 整数精确，覆盖 u16 参数。f32; exact for integers up to
                ///< 2^24, which covers u16 parameters.
};

// 字段本身占 24B（`value` 结束于 24，ABI 不补尾）。页内为 payload 预留 28B，多出的
// 4B 是 `Region` 的显式填充：把 payload 做成 28B 会让 `Region` 变成 36B、`seq` 落到
// 32，反而违约。
// The fields occupy 24B (the last one ends at 24 and the ABI adds no tail padding).
// The page reserves 28B for the payload; the extra 4B is explicit padding on
// `Region`, because padding the payload to 28B would make `Region` 36B and put `seq`
// at 32.
static_assert(sizeof(RegionPayload) == 24, "RegionPayload fields are 24B");
static_assert(offsetof(RegionPayload, aim_x) == 4, "RegionPayload::aim_x offset pinned");
static_assert(offsetof(RegionPayload, aim_y) == 8, "RegionPayload::aim_y offset pinned");
static_assert(offsetof(RegionPayload, cmd) == 12, "RegionPayload::cmd offset pinned");
static_assert(offsetof(RegionPayload, param_id) == 16,
              "RegionPayload::param_id offset pinned");
static_assert(offsetof(RegionPayload, value) == 20, "RegionPayload::value offset pinned");

/**
 * @struct Region
 * @brief 参考/命令区：Linux 单写者。Reference/command region: a single writer on Linux.
 *
 * 写者写 payload 字段后 `seq + 1`（release）；读者读 `seq`（acquire）→ 拷 payload →
 * 复读 `seq`，不一致则重试。
 * The writer stores the payload fields and then bumps `seq` (release); the reader
 * acquires `seq`, copies the payload, re-reads `seq` and retries on a mismatch.
 */
struct Region
{
  RegionPayload payload;      ///< 有效载荷。Payload.
  uint32_t payload_pad;       ///< 尾部留白，把 `seq` 顶到 28。Trailing slack that puts
                              ///< `seq` at 28.
  std::atomic<uint32_t> seq;  ///< 发布索引：每次写 region +1。Publish index: +1 on every
                              ///< region write.

  /// @brief CMD_TRIGGER：本帧触发一次。Trigger once for this frame.
  static constexpr uint8_t CMD_TRIGGER = 1;
  /// @brief CMD_PARAM：写一个 C606 侧参数。Write one C606-side parameter.
  static constexpr uint8_t CMD_PARAM = 2;
};

static_assert(sizeof(Region) == 32, "Region layout pinned");
static_assert(offsetof(Region, payload_pad) == 24, "Region::payload_pad offset pinned");
static_assert(offsetof(Region, seq) == 28, "Region::seq offset pinned");
static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
              "Region::seq must occupy exactly 4 bytes for the pinned offset to hold");

/**
 * @struct AccessUnit
 * @brief 相机访问单元槽（mailbox）：magic / CRC / 借还语义。
 *        Camera access-unit slot (mailbox): magic, CRC and borrow semantics.
 *
 * 单槽「最新帧」语义：写者填 payload 后 release 推 `seq`，读者比对 `seq` 后取走并清
 * `ready`。读者慢时写者覆写旧帧而不是阻塞，与 keep-latest 订阅一致。
 * Latest-frame semantics of a single slot: the writer fills the payload and releases
 * `seq`, and the reader takes the frame once it matches and clears `ready`. A slow
 * reader loses the old frame to an overwrite instead of blocking the writer, matching
 * a keep-latest subscription.
 */
struct AccessUnit
{
  uint32_t magic;   ///< 必须为 `ACCESS_UNIT_MAGIC`。Must equal `ACCESS_UNIT_MAGIC`.
  uint32_t format;  ///< 编码格式，见下方枚举值。Encoding format; see the values below.
  uint32_t width;   ///< 帧宽。Frame width.
  uint32_t height;  ///< 帧高。Frame height.
  std::atomic<uint32_t> seq;     ///< 内容序号（release store）。Content sequence.
  std::atomic<uint32_t> length;  ///< 有效字节数，≤ `MAILBOX_BYTES`。Valid byte count, no
                                 ///< more than `MAILBOX_BYTES`.
  std::atomic<uint32_t> ready;   ///< 1 = 有新帧可取；读者取走后置 0。1 = a frame is
                                 ///< available; cleared by the reader.
  std::atomic<uint32_t> crc32;   ///< payload CRC32（0 = 未校验）。CRC32 of the payload
                                 ///< (0 = unchecked).
  uint32_t reserved;             ///< 对齐与后续扩展留白。Alignment and future use.
  uint8_t payload[MAILBOX_BYTES];  ///< 访问单元字节。Access-unit bytes.

  /// @brief 单帧最大字节数。Maximum bytes of one frame.
  static constexpr uint32_t MAX_BYTES = MAILBOX_BYTES;

  /// @brief 编码格式枚举值：未知。Encoding format: unknown.
  static constexpr uint32_t FORMAT_UNKNOWN = ACCESS_UNIT_FORMAT_UNKNOWN;
  /// @brief 编码格式枚举值：H.264 Annex-B。Encoding format: H.264 Annex-B.
  static constexpr uint32_t FORMAT_H264_ANNEX_B = ACCESS_UNIT_FORMAT_H264_ANNEX_B;
};

/**
 * @brief 计算页内各区的字节偏移，供双端一致引用。
 *        Compute the byte offsets of every region inside one page so both cores
 *        reference the same numbers.
 */
inline constexpr size_t TelemetryOffset()
{
  // 第 0..3 字节是 magic，第 4..7 字节是 page size 自描述，遥测区从 8 起。
  return 8;
}

/// @brief 参考/命令区的页内偏移。In-page offset of the reference region.
inline constexpr size_t RegionOffset() { return 4096 - sizeof(Region); }

/**
 * @brief 页头部：magic + 自描述大小。Page header: magic plus self-described size.
 */
struct PageHeader
{
  uint32_t magic;      ///< `PAGE_MAGIC`。Page magic.
  uint32_t page_size;  ///< 本契约的页大小（`PAGE_SIZE`）。Page size of this contract.
};

static_assert(offsetof(PageHeader, page_size) == 4, "PageHeader layout pinned");
static_assert(TelemetryOffset() >= sizeof(PageHeader),
              "telemetry must not overlap header");
static_assert(TelemetryOffset() + sizeof(TelemetryRing) <= RegionOffset(),
              "telemetry ring and region must not overlap");

/**
 * @class PageBase
 * @brief 页的基类：内存校验与只读探测。Base class of one page: validation and probing.
 *
 * 不拥有内存：页由 Linux 侧 `/dev/mem` 非缓存映射，或由 C606 侧链接脚本分配。地址经
 * yaml 配置注入，不进模块构造参数。
 * Does not own the memory: a page is either mapped non-cached from `/dev/mem` on Linux
 * or allocated by the C606 linker script, and the address is injected through yaml
 * configuration rather than a module constructor argument.
 */
class PageBase
{
 public:
  PageBase() = default;

  /**
   * @brief 绑定一个已映射的页。Bind an already-mapped page.
   * @param addr 页起始地址（4 KiB 对齐）；`nullptr` 表示未绑定。Page base address (4 KiB
   *             aligned); `nullptr` means unbound.
   */
  explicit PageBase(void* addr) : page_(static_cast<uint8_t*>(addr)) {}

  /**
   * @brief 页是否已绑定。Whether a page is bound.
   */
  [[nodiscard]] bool Valid() const { return page_ != nullptr; }

  /**
   * @brief 页起始地址。Page base address.
   */
  [[nodiscard]] uint8_t* Data() const { return page_; }

  /**
   * @brief 校验映射到的内存是否属于本契约。Validate that the mapped memory belongs to
   *        this contract.
   * @return 见 `PageMagicKind`。See `PageMagicKind`.
   */
  [[nodiscard]] PageMagicKind Check() const
  {
    if (page_ == nullptr)
    {
      return PageMagicKind::UNFORMATTED;
    }

    const auto* nodes = reinterpret_cast<const PageHeader*>(Data());
    if (nodes->magic == PAGE_MAGIC && nodes->page_size == PAGE_SIZE)
    {
      return PageMagicKind::FORMATTED;
    }

    // 全零页面视为「刚上电 / 映射错地址」；其余内容视为外来数据。
    // An all-zero page is treated as cold or mis-mapped; anything else is foreign.
    for (size_t i = 0; i < PAGE_SIZE; ++i)
    {
      if (Data()[i] != 0)
      {
        return PageMagicKind::FOREIGN;
      }
    }
    return PageMagicKind::UNFORMATTED;
  }

  /**
   * @brief 把一页格式化为本契约（清零 + 写 magic）。Format one page for this contract
   *        (clear, then stamp the magic).
   *
   * 只在首次上电或重新分配地址后执行。
   * Run only on first power-up or after re-addressing.
   */
  void Format()
  {
    if (page_ == nullptr)
    {
      return;
    }

    std::memset(page_, 0, PAGE_SIZE);
    auto* nodes = reinterpret_cast<PageHeader*>(Data());
    nodes->magic = PAGE_MAGIC;
    nodes->page_size = PAGE_SIZE;
  }

  /**
   * @brief 清空遥测与参考区的发布索引（保留 magic）。
   *        Clear the publish indices of the telemetry and reference regions (keeps the
   *        magic).
   *
   * 任一侧重启后使用，避免读到上次运行的历史。
   * Used after either side restarts so no history from the previous run is read.
   */
  void ClearHistory()
  {
    if (page_ == nullptr)
    {
      return;
    }

    TelemetryRing* telemetry = nullptr;
    Region* region = nullptr;
    Split(&telemetry, &region);
    telemetry->head.store(0, std::memory_order_release);
    region->seq.store(0, std::memory_order_release);
  }

 protected:
  /**
   * @brief 定位页内的遥测区与参考区。Locate the telemetry and reference regions.
   * @param telemetry 输出：遥测区地址。Output: telemetry region.
   * @param region 输出：参考区地址。Output: reference region.
   */
  void Split(TelemetryRing** telemetry, Region** region) const
  {
    ASSERT(page_ != nullptr);
    *telemetry = reinterpret_cast<TelemetryRing*>(page_ + TelemetryOffset());
    *region = reinterpret_cast<Region*>(page_ + RegionOffset());
  }

  uint8_t* page_ = nullptr;  ///< 页起始地址，不拥有内存。Page base address; memory is
                             ///< not owned.
};

/**
 * @class Telemetry
 * @brief 遥测区的生产者视图（C606 控制环）。Producer view of the telemetry region (the
 *        C606 control loop).
 *
 * 写入是「先 payload、后 head」的 release store：RVWMO 保证同 hart 的 store 次序，
 * 无需 fence。
 * Writing is payload first and then a release store of `head`: RVWMO guarantees
 * same-hart store order, so no fence is needed.
 */
class Telemetry
{
 public:
  Telemetry() = default;

  /**
   * @brief 从页基址构造。Construct from a page base address.
   */
  explicit Telemetry(void* addr)
      : head_(reinterpret_cast<std::atomic<uint32_t>*>(static_cast<uint8_t*>(addr) +
                                                       TelemetryOffset() +
                                                       offsetof(TelemetryRing, head)))
  {
  }

  Telemetry(const Telemetry&) = delete;
  Telemetry& operator=(const Telemetry&) = delete;
  Telemetry(Telemetry&&) = delete;
  Telemetry& operator=(Telemetry&&) = delete;

  /**
   * @brief 写一条采样并推发布索引。Write one sample and push the publish index.
   * @param sample 待发布采样。Sample to publish.
   * @return 写入后的 `head`（即该采样的区间上界）。The resulting `head`, which is the
   *         exclusive upper bound of that sample's range.
   */
  uint32_t Write(const Sample& sample)
  {
    ASSERT(head_ != nullptr);

    // head_ 指向页内原子对象，物理上就是 mmap 出来的非缓存内存。
    // head_ points at an in-page atomic object backed by non-cached mapped memory.
    uint32_t head = head_->load(std::memory_order_relaxed);
    const auto* ring = reinterpret_cast<const Sample*>(
        reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
    auto* slot = const_cast<Sample*>(&ring[head % TELEMETRY_SLOTS]);
    *slot = sample;
    head_->store(head + 1, std::memory_order_release);
    return head + 1;
  }

  /**
   * @brief 当前已发布条数。Current published count.
   */
  [[nodiscard]] uint32_t Head() const
  {
    return head_ == nullptr ? 0 : head_->load(std::memory_order_acquire);
  }

  /**
   * @brief 取最新一条采样。Read the latest sample.
   * @param sample 输出：最新采样。Output: latest sample.
   * @return 有数据返回 `true`；`head == 0` 返回 `false`。`true` when data exists; `false`
   *         when `head == 0`.
   */
  [[nodiscard]] bool Latest(Sample* sample) const
  {
    if (sample == nullptr || head_ == nullptr)
    {
      return false;
    }

    const uint32_t head = head_->load(std::memory_order_acquire);
    if (head == 0)
    {
      return false;
    }

    const auto* ring = reinterpret_cast<const Sample*>(
        reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
    *sample = ring[(head - 1) % TELEMETRY_SLOTS];
    return true;
  }

  /**
   * @brief 扫描 `(last_seen, head]` 区间。Scan the range `(last_seen, head]`.
   *
   * 只解析 `head - last_seen <= 64` 的区间；读者落后更多时写者已覆写那些槽，整段丢弃
   * 并返回 `RingScan::GAP`。
   * Only a range with `head - last_seen <= 64` is parsed; further behind, the writer
   * has already overwritten those slots, so the whole range is dropped with
   * `RingScan::GAP`.
   *
   * @param last_seen 读者上次消费到的条数（初值 0）。Count the reader consumed last.
   * @param scan 输出：区间结果。Output: range result.
   * @param next 输出：下次调用应传入的 `last_seen`。Output: the `last_seen` for the next
   *             call.
   * @param samples 接收缓冲，可为 `nullptr`（只判定区间与 gap）。Receive buffer;
   * `nullptr` only decides the range and the gap.
   * @param capacity `samples` 的槽数。Slot count of `samples`.
   * @return 解析出的采样条数；`scan != RingScan::DATA` 时为 0。Parsed sample count; 0
   *         when `scan != RingScan::DATA`.
   */
  uint32_t Since(uint32_t last_seen, RingScan* scan, uint32_t* next, Sample* samples,
                 uint32_t capacity) const
  {
    ASSERT(scan != nullptr);
    ASSERT(next != nullptr);
    ASSERT(head_ != nullptr);

    const uint32_t head = head_->load(std::memory_order_acquire);

    if (head == last_seen)
    {
      *scan = RingScan::IDLE;
      *next = head;
      return 0;
    }

    if (head - last_seen > TELEMETRY_SLOTS)
    {
      *scan = RingScan::GAP;
      *next = head;
      return 0;
    }

    const uint32_t count = head - last_seen;

    // 写者可能在区间判定与拷贝之间继续推进并覆写最旧槽，所以按最新优先倒序拷贝，
    // 让区间里最旧的一条尽早读到，撕裂窗口最小。
    // The writer may advance between the range decision and the copies, overwriting the
    // oldest slots, so the range is copied newest first to read its oldest sample as
    // early as possible.
    const auto* ring = reinterpret_cast<const Sample*>(
        reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
    if (samples != nullptr)
    {
      const uint32_t limit = count < capacity ? count : capacity;
      for (uint32_t i = 0; i < limit; ++i)
      {
        samples[i] = ring[(head - 1 - i) % TELEMETRY_SLOTS];
      }
      // 倒序读入后翻正，调用者拿到正序区间。
      // Reverse the newest-first copies so the caller sees the range in order.
      for (uint32_t i = 0, j = (limit == 0) ? 0 : limit - 1; i < j; ++i, --j)
      {
        const Sample tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }

    *scan = RingScan::DATA;
    *next = head;
    return count;
  }

  /**
   * @brief 丢弃既往历史，返回写者当前位置。Drop past history and return the writer
   *        position.
   *
   * broadcast-drop-old 订阅模式使用（SD 慢时丢最旧 + gap）。
   * Used by a broadcast-drop-old subscriber (drop oldest and mark a gap when the SD
   * card is slow).
   */
  [[nodiscard]] uint32_t SeekToHead() const { return Head(); }

 private:
  std::atomic<uint32_t>* head_ = nullptr;  ///< 页内 `ring.head` 的原子视图。Atomic view
                                           ///< of the in-page `ring.head`.
};

/**
 * @class Reference
 * @brief 参考/命令区的单写者视图（Linux 侧）。Single-writer view of the reference and
 *        command region, used on the Linux side.
 *
 * 读者读 `seq`（acquire）→ 拷 payload → 复读 `seq`，不一致则重试；acquire 不可省，
 * RVWMO 允许 load-load 重排。
 * The reader acquires `seq`, copies the payload, re-reads `seq` and retries on a
 * mismatch. The acquire cannot be dropped: RVWMO permits load-load reordering.
 */
class Reference
{
 public:
  Reference() = default;

  /**
   * @brief 从页基址构造。Construct from a page base address.
   */
  explicit Reference(void* addr)
      : region_(reinterpret_cast<Region*>(static_cast<uint8_t*>(addr) + RegionOffset()))
  {
  }

  Reference(const Reference&) = delete;
  Reference& operator=(const Reference&) = delete;
  Reference(Reference&&) = delete;
  Reference& operator=(Reference&&) = delete;

  /**
   * @brief 读一个自洽的 region 快照。Read one coherent region snapshot.
   *
   * 正常情况下是两条 acquire load 加一次拷贝；只有该次读可能撕裂时才重试。输出的
   * `Region` 是调用者持有的快照，其 `seq` 为普通值而非页内发布索引。
   * The normal path is two acquire loads plus one copy, retrying only when the read may
   * be torn. The returned `Region` is a caller-owned snapshot whose `seq` is a plain
   * value rather than the in-page publish index.
   *
   * @param out 输出：region 快照。Output: region snapshot.
   * @param retries 撕裂重试上限。Tear-retry limit.
   * @return 见 `RegionScan`。See `RegionScan`.
   */
  [[nodiscard]] RegionScan Read(Region* out, uint32_t retries = 8) const
  {
    ASSERT(out != nullptr);
    ASSERT(region_ != nullptr);

    for (uint32_t attempt = 0; attempt < retries; ++attempt)
    {
      const uint32_t before = region_->seq.load(std::memory_order_acquire);
      out->payload = region_->payload;
      const uint32_t after = region_->seq.load(std::memory_order_acquire);
      if (before == after)
      {
        out->seq.store(after, std::memory_order_relaxed);
        return RegionScan::CURRENT;
      }
    }

    // 重试上限内未读到自洽快照（写者持续发布）：返回最后一次拷贝并标记 BUSY。
    // No coherent snapshot within the retry budget (the writer keeps publishing):
    // return the last copy tagged BUSY.
    out->payload = region_->payload;
    out->seq.store(region_->seq.load(std::memory_order_acquire),
                   std::memory_order_relaxed);
    return RegionScan::BUSY;
  }

  /**
   * @brief 只读当前发布索引。Read only the current publish index.
   */
  [[nodiscard]] uint32_t Seq() const
  {
    return region_ == nullptr ? 0 : region_->seq.load(std::memory_order_acquire);
  }

  /**
   * @brief 写一个 region（先 payload，后 `seq + 1`）。
   *        Write one region: the payload first, then `seq + 1`.
   *
   * Linux 每帧写一次（found/aim）；Razver POLL 回传的命令越过白名单后同样走这里。
   * Linux writes once per frame (found/aim); a command returned by the Razver POLL path
   * goes through the same call once it passes the whitelist.
   *
   * @param payload 待写 payload。Payload to write.
   * @return 写入后的 `seq`。The resulting `seq`.
   */
  uint32_t Write(const RegionPayload& payload)
  {
    ASSERT(region_ != nullptr);

    region_->payload = payload;
    const uint32_t next = region_->seq.load(std::memory_order_relaxed) + 1;
    region_->seq.store(next, std::memory_order_release);
    return next;
  }

  /**
   * @brief 写一帧视觉参考。Write one frame of visual reference.
   * @param found 是否有目标。Whether a target was found.
   * @param aim_x 归一化中心 x。Normalised centre x.
   * @param aim_y 归一化中心 y。Normalised centre y.
   */
  uint32_t WriteAim(bool found, float aim_x, float aim_y)
  {
    RegionPayload payload = {};
    payload.found = found ? 1U : 0U;
    payload.aim_x = aim_x;
    payload.aim_y = aim_y;
    return Write(payload);
  }

  /**
   * @brief 直接取页内 region 指针（C606 侧热路径直读）。Get the in-page region pointer
   *        for the C606 hot path that reads it in place.
   */
  [[nodiscard]] const Region* Raw() const { return region_; }

 private:
  Region* region_ = nullptr;  ///< 页内参考区。In-page reference region.
};

/**
 * @class SharedPage
 * @brief 一个已映射页的完整视图：遥测 + 参考。Complete view of one mapped page:
 *        telemetry plus reference.
 *
 * C606 侧控制环直接使用；Linux 侧由 `LinuxSharedPage` 把这个页包成 Topic。
 * Used directly by the C606 control loop; on the Linux side `LinuxSharedPage` wraps
 * the page into a topic.
 */
class SharedPage : public PageBase
{
 public:
  SharedPage() = default;

  /**
   * @brief 绑定一个已映射页。Bind one already-mapped page.
   * @param addr 页起始地址；`nullptr` 表示未绑定。Page base address; `nullptr` means
   *             unbound.
   */
  explicit SharedPage(void* addr) : PageBase(addr) {}

  /**
   * @brief 页是否可用（已绑定且魔术字正确）。Whether the page is usable (bound and the
   *        magic matches).
   */
  [[nodiscard]] bool Ready() const { return Check() == PageMagicKind::FORMATTED; }

  /**
   * @brief 遥测区生产者视图（C606 控制环写）。Telemetry producer view (written by the
   *        C606 control loop).
   */
  [[nodiscard]] Telemetry TelemetryWriter() { return Telemetry(Data()); }

  /**
   * @brief 遥测区生产者视图（const 重载）。Telemetry producer view (const overload).
   */
  [[nodiscard]] Telemetry TelemetryWriter() const
  {
    return Telemetry(const_cast<uint8_t*>(Data()));
  }

  /**
   * @brief 遥测区消费者视图（LinuxSharedPage drain 用）。Telemetry consumer view
   *        (used by the LinuxSharedPage drain).
   */
  [[nodiscard]] Telemetry TelemetryReader() const { return TelemetryWriter(); }

  /**
   * @brief 参考/命令视图。Reference/command view.
   */
  [[nodiscard]] Reference Region() { return Reference(Data()); }

  /**
   * @brief 参考/命令视图（const 重载）。Reference/command view (const overload).
   */
  [[nodiscard]] Reference Region() const
  {
    return Reference(const_cast<uint8_t*>(Data()));
  }

  /**
   * @brief 便捷入口：写一条遥测采样。Convenience: write one telemetry sample.
   */
  uint32_t WriteSample(const Sample& sample) { return TelemetryWriter().Write(sample); }

  /**
   * @brief 便捷入口：取最新一条遥测采样。Convenience: read the latest telemetry sample.
   */
  [[nodiscard]] bool Latest(Sample* sample) const
  {
    return TelemetryReader().Latest(sample);
  }
};

/**
 * @class AccessUnitPage
 * @brief 独立的访问单元页（相机 mailbox），与遥测/参考页分开映射。
 *        Separate access-unit page (the camera mailbox), mapped apart from the
 *        telemetry/reference page.
 *
 * 只写页内的头部字段，不清 512 KiB 的 payload 区：那一次清零在页首次映射时做即可。
 * Only the header fields are written; the 512 KiB payload area is cleared once when
 * the page is first mapped.
 */
class AccessUnitPage : public PageBase
{
 public:
  AccessUnitPage() = default;

  /**
   * @brief 绑定一个已映射的访问单元页。Bind one already-mapped access-unit page.
   */
  explicit AccessUnitPage(void* addr) : PageBase(addr) {}

  /**
   * @struct View
   * @brief 一帧访问单元的只读视图。Read-only view of one access unit.
   */
  struct View
  {
    const uint8_t* data = nullptr;  ///< payload 起始地址。Payload base address.
    uint32_t length = 0;            ///< 有效字节数。Valid byte count.
    uint32_t format = 0;            ///< 编码格式。Encoding format.
    uint32_t width = 0;             ///< 帧宽。Frame width.
    uint32_t height = 0;            ///< 帧高。Frame height.
    uint32_t seq = 0;               ///< 内容序号。Content sequence.

    /// @brief 视图是否有效。Whether the view is valid.
    [[nodiscard]] bool Valid() const { return data != nullptr && length > 0; }
  };

  /**
   * @brief 访问单元区在页内的偏移。In-page offset of the access-unit region.
   */
  static constexpr size_t Offset() { return 0; }

  /**
   * @brief 格式化访问单元页（写 magic + 头部）。Format the access-unit page (magic plus
   *        header).
   */
  void Format()
  {
    if (Data() == nullptr)
    {
      return;
    }

    auto* slot = Slot();
    slot->magic = ACCESS_UNIT_MAGIC;
    slot->format = AccessUnit::FORMAT_UNKNOWN;
    slot->width = 0;
    slot->height = 0;
    slot->seq.store(0, std::memory_order_release);
    slot->length.store(0, std::memory_order_release);
    slot->ready.store(0, std::memory_order_release);
    slot->crc32.store(0, std::memory_order_release);
    slot->reserved = 0;
  }

  /**
   * @brief 校验访问单元页的魔术字。Validate the access-unit page magic.
   */
  [[nodiscard]] PageMagicKind Check() const
  {
    if (Data() == nullptr)
    {
      return PageMagicKind::UNFORMATTED;
    }
    return Slot()->magic == ACCESS_UNIT_MAGIC ? PageMagicKind::FORMATTED
                                              : PageMagicKind::UNFORMATTED;
  }

  /**
   * @brief 发布一帧访问单元（相机侧，持有 VENC buffer 时调用）。
   *        Publish one access unit, called on the camera side while the VENC buffer is
   *        held.
   *
   * @param data 访问单元字节。Access-unit bytes.
   * @param length 字节数，不超过 `MAILBOX_BYTES`。Byte count, no more than
   *               `MAILBOX_BYTES`.
   * @param format 编码格式。Encoding format.
   * @param width 帧宽。Frame width.
   * @param height 帧高。Frame height.
   * @param crc32 payload CRC32，0 表示不校验。Payload CRC32; 0 means unchecked.
   * @return 写入后的 `seq`；越界、空指针或未绑定时为 0。The resulting `seq`; 0 when
   *         oversized, null or unbound.
   */
  uint32_t Publish(const void* data, uint32_t length, uint32_t format, uint32_t width,
                   uint32_t height, uint32_t crc32 = 0)
  {
    if (Data() == nullptr || data == nullptr || length > AccessUnit::MAX_BYTES)
    {
      return 0;
    }

    auto* slot = Slot();
    std::memcpy(slot->payload, data, length);
    slot->format = format;
    slot->width = width;
    slot->height = height;
    slot->crc32.store(crc32, std::memory_order_relaxed);
    slot->length.store(length, std::memory_order_relaxed);
    slot->ready.store(1, std::memory_order_relaxed);

    // release：保证 payload/头部字段对取帧者可见再公开 seq。
    // release: make the payload and header fields visible before publishing seq.
    const uint32_t next = slot->seq.load(std::memory_order_relaxed) + 1;
    slot->seq.store(next, std::memory_order_release);
    return next;
  }

  /**
   * @brief 取一帧访问单元（Linux 侧，`camera_mailbox` 借还语义）。
   *        Take one access unit on the Linux side, with the `camera_mailbox` borrow
   *        semantics.
   *
   * 返回的指针直接指向非缓存页内，保持到下次 `Publish()` 之前；取走后 `ready` 已清 0，
   * 调用者应尽快消费（拷贝或编码）。
   * The pointer points straight into the non-cached page and stays valid until the next
   * `Publish()`; `ready` is already cleared, so the caller should consume the frame
   * promptly (copy or encode).
   *
   * @param crc32 输出：本帧 payload CRC32。Output: payload CRC32 of this frame.
   * @param retries 撕裂重试上限。Tear-retry limit.
   * @return 未绑定、无新帧或撕裂未消除时返回空视图。An empty view when unbound, no frame
   *         is ready, or a tear could not be resolved.
   */
  [[nodiscard]] View Acquire(uint32_t* crc32 = nullptr, uint32_t retries = 8)
  {
    View view = {};
    if (Data() == nullptr)
    {
      return view;
    }

    auto* slot = Slot();
    if (slot->ready.load(std::memory_order_acquire) == 0)
    {
      return view;
    }

    for (uint32_t attempt = 0; attempt < retries; ++attempt)
    {
      const uint32_t before = slot->seq.load(std::memory_order_acquire);
      const uint32_t length = slot->length.load(std::memory_order_acquire);
      if (length > AccessUnit::MAX_BYTES)
      {
        break;
      }
      const uint32_t after = slot->seq.load(std::memory_order_acquire);
      if (before == after)
      {
        view.data = slot->payload;
        view.length = length;
        view.format = slot->format;
        view.width = slot->width;
        view.height = slot->height;
        view.seq = before;
        slot->ready.store(0, std::memory_order_release);
        if (crc32 != nullptr)
        {
          *crc32 = slot->crc32.load(std::memory_order_relaxed);
        }
        return view;
      }
    }

    return view;
  }

 private:
  [[nodiscard]] AccessUnit* Slot() const
  {
    return reinterpret_cast<AccessUnit*>(const_cast<uint8_t*>(Data()) + Offset());
  }
};

}  // namespace LibXR
