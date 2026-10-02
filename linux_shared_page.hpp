#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "libxr_time.hpp"
#include "shared_page.hpp"
#include "timebase.hpp"
#include "topic.hpp"

/**
 * @file linux_shared_page.hpp
 * @brief 共享页在 Linux 侧的 Topic 适配（对齐 `LinuxSharedTopic` 的做法）。
 *        Linux-side topic adapter for a shared page, matching the
 *        `LinuxSharedTopic` idiom.
 *
 * 页只是 `SharedPage` 后端里的传输：模块在两侧看到的都是 topic，页知识只留在这个
 * 适配器里。C606 侧不需要本文件——那边直接拿 `SharedPage` 用（无 Topic、无适配层）。
 * A page is only the transport behind `SharedPage`: modules see topics on both
 * sides and all page knowledge stays in this adapter. The C606 side does not need
 * this header; it uses `SharedPage` directly, with no topic and no adapter.
 *
 * 具体到两条方向：
 *
 * - **遥测（C606 → Linux）**：`Poll()` 以配置的节律（默认 1kHz）drain 遥测区间，
 *   把新区间打包成**一条** `TelemetryBatch` 发布到 `telemetry` topic。逐条发布会让
 *   每条采样都占一次订阅通知；批量发布让发布次数正比于控制环推进次数，与“区间”
 *   这个契约单位一致。话题是广播的：`Recorder` 用 broadcast-full 全量落 SD，
 *   `RazverMaster` 取最新一组组 metadata。
 * - **参考/命令（Linux → C606）**：`WriteAim()` 每帧写一次，`WriteParam()` 写
 *   Razver POLL 回传并已过大核白名单过滤的参数；两者都是 `seq + 1`。C606 控制环
 *   直接读 region，不经本适配器。
 *
 * - **Telemetry (C606 -> Linux)**: `Poll()` drains the telemetry range at the
 *   configured cadence (1kHz by default) and publishes the new range as **one**
 *   `TelemetryBatch` on the `telemetry` topic. Publishing sample by sample would
 *   spend one subscriber notification per sample; batching makes the publish count
 *   proportional to how often the control loop advanced, which is the unit the
 *   contract actually uses. The topic broadcasts, so `Recorder` takes it
 *   broadcast-full into SD and `RazverMaster` keeps the latest batch for metadata.
 * - **Reference/command (Linux -> C606)**: `WriteAim()` once per frame and
 *   `WriteParam()` for a whitelist-filtered Razver POLL parameter; both bump
 *   `seq`. The C606 control loop reads the region directly and never goes through
 *   this adapter.
 *
 * 页地址经 yaml 配置注入（Linux 侧），不进模块构造参数：模块拿到的是已经映射好的
 * 页，本适配器不碰 `/dev/mem`。非缓存映射（`/dev/mem` + `O_SYNC`，或 SG200x 的
 * `mmap` 非缓存属性）由初始化代码完成，因为它需要 root，且是平台配置而不是模块
 * 行为。
 * The page address is injected through yaml configuration on the Linux side and is
 * never a module constructor argument: the module receives an already-mapped page,
 * and this adapter does not touch `/dev/mem`. The non-cached mapping (`/dev/mem`
 * with `O_SYNC`, or the SG200x non-cached `mmap` attribute) is set up by
 * initialisation code, because it needs root and is platform configuration rather
 * than module behaviour.
 */

namespace SharedPageXR
{
/**
 * @brief 遥测 topic 的缺省名称。Default name of the telemetry topic.
 */
inline constexpr const char* TELEMETRY_TOPIC_NAME = "telemetry";

/**
 * @brief 遥测 topic 的缺省域。Default domain of the telemetry topic.
 */
inline constexpr const char* SHARED_PAGE_DOMAIN_NAME = "shared_page_xr";

/**
 * @struct TelemetryBatch
 * @brief 遥测 topic 的载荷：一次 drain 覆盖的区间。Telemetry topic payload: the range
 *        one drain covered.
 *
 * `head` 是本批最后一条之后的发布索引，订阅者据此保持自己的 `last_seen`；`gap`
 * 表示更早的历史已被写者覆写（有界历史的固有竞态），订阅者应记一段无效区间而不是
 * 试图补齐。`ring` 里的槽位是值拷贝——订阅回调返回后页可能已被覆写，所以载荷必须
 * 自持数据。
 * `head` is the publish index after the last sample of this batch, which is how a
 * subscriber keeps its own `last_seen`; `gap` marks that older history was already
 * overwritten (the inherent race of a bounded history), so a subscriber records an
 * invalid interval instead of trying to fill it in. The `ring` slots are copied by
 * value: the page may be overwritten after the callback returns, so the payload
 * must own its data.
 */
struct TelemetryBatch
{
  Sample ring[TELEMETRY_SLOTS] = {};  ///< 区间内的采样，按 `(head0, head]` 正序。Samples of
                                     ///< this range in ascending order.
  uint32_t count = 0;   ///< 有效条数，≤ `TELEMETRY_SLOTS`。Valid sample count.
  uint32_t head = 0;    ///< 本批之后的发布索引。Publish index after this batch.
  uint8_t gap = 0;      ///< 1 = 本批之前有被覆写的区间。1 = history before this batch was
                       ///< overwritten.
  uint8_t reserved[3] = {};  ///< 对齐留白。Alignment padding.
};

/// @brief `TelemetryBatch` 的字节数，作为 topic payload 契约的一部分。
///        Byte size of `TelemetryBatch`, part of the topic payload contract.
inline constexpr size_t TELEMETRY_BATCH_BYTES = sizeof(TelemetryBatch);

static_assert(sizeof(TelemetryBatch) == sizeof(Sample) * TELEMETRY_SLOTS + 16,
              "TelemetryBatch layout pinned");

/**
 * @struct LinuxSharedPageConfig
 * @brief Linux 侧共享页适配器的创建配置。Creation config of the Linux-side shared-page
 *        adapter.
 */
struct LinuxSharedPageConfig
{
  uint32_t drain_period_us = 1000;  ///< drain 节律，默认 1kHz。Drain cadence, 1kHz by
                                    ///< default.
  const char* topic_name = TELEMETRY_TOPIC_NAME;  ///< 遥测 topic 名称。Telemetry topic
                                                 ///< name.
  const char* domain_name = SHARED_PAGE_DOMAIN_NAME;  ///< topic 域名称。Topic domain
                                                      ///< name.
};

/**
 * @class LinuxSharedPage
 * @brief Linux 侧的共享页：drain 遥测区间并发 topic，写参考区。
 *        Linux-side shared page: drain the telemetry range into a topic and write
 *        the reference region.
 *
 * 不拥有页内存，也不拥有 topic：两者都由初始化代码按 yaml 配置构造后注入，与
 * `LinuxSharedTopic` 由调用者持有句柄的做法一致。
 * It owns neither the page memory nor the topic: both are constructed by
 * initialisation code from yaml configuration and injected here, matching how a
 * `LinuxSharedTopic` handle is owned by its caller.
 */
class LinuxSharedPage
{
 public:
  /**
   * @brief 绑定页与 topic。Bind the page and the topic.
   * @param page 已映射的页视图。Already-mapped page view.
   * @param topic 遥测 topic（payload 为 `TelemetryBatch`）。Telemetry topic whose
   *              payload is `TelemetryBatch`.
   * @param config 创建配置。Creation config.
   */
  LinuxSharedPage(SharedPage page, LibXR::Topic topic,
                  const LinuxSharedPageConfig& config = {})
      : page_(page), topic_(topic), period_us_(config.drain_period_us)
  {
    // 不在这里读时钟：`last_drain_us_ = 0` 让第一次 `Poll()` 只负责确立节律基准，
    // 也避免与调用者自己的时钟注入（测试）互相干扰。此时若页上已有待发布区间，
    // 第一次 `Poll()` 会把它发出去，这本就是启动时该有的行为。
    // Do not read a clock here: `last_drain_us_ = 0` makes the first `Poll()` the
    // one that establishes the cadence baseline and keeps caller-injected clocks
    // (tests) independent. If the page already holds an unpublished range, that
    // first `Poll()` publishes it, which is what startup should do anyway.
  }

  LinuxSharedPage(const LinuxSharedPage&) = delete;
  LinuxSharedPage& operator=(const LinuxSharedPage&) = delete;

  /**
   * @brief 页是否处在可用状态（已绑定且魔术字正确）。
   *        Whether the page is usable (bound and the magic matches).
   */
  [[nodiscard]] bool Ready() const { return page_.Ready(); }

  /**
   * @brief 参考/命令区视图（供写 aim 与参数）。Reference/command view (for writing the
   *        aim and parameters).
   */
  [[nodiscard]] Reference Region() { return page_.Region(); }

  /**
   * @brief 按节律 drain 一次（正常路径每周期调用一次）。
   *        Drain once when the cadence is due (the normal path calls this every
   *        cycle).
   *
   * 未到节律时是两次时间读取加一次比较的早退，不发布。
   * When the cadence is not due this is an early return after two clock reads and
   * one comparison, with no publish.
   *
   * @param now_us 当前时间（微秒），取自 `LibXR::Timebase`；缺省由内部读取，测试可注入。
   *               Current time in microseconds from `LibXR::Timebase`; read
   *               internally by default, injectable for tests.
   * @return 本次发布了一组遥测返回 `true`。`true` when a batch was published.
   */
  bool Poll(uint64_t now_us = UINT64_MAX)
  {
    if (now_us == UINT64_MAX)
    {
      now_us = static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
    }

    if (now_us - last_drain_us_ < period_us_)
    {
      return false;
    }
    last_drain_us_ = now_us;

    TelemetryBatch batch = {};
    if (Drain(&batch) == 0 && batch.gap == 0)
    {
      return false;
    }

    LibXR::MicrosecondTimestamp timestamp(now_us);
    topic_.Publish(batch, timestamp);
    return true;
  }

  /**
   * @brief 立即 drain 并填充一组（不发布；测试与 `RazverMaster` 组 metadata 用）。
   *        Drain now into one batch without publishing (used by tests and by
   *        `RazverMaster` when it assembles metadata).
   *
   * @param batch 输出：本组遥测。Output: this telemetry batch.
   * @return 本组采样条数（0 表示无新数据）。Sample count of this batch (0 means no new
   *         data).
   */
  uint32_t Drain(TelemetryBatch* batch)
  {
    ASSERT(batch != nullptr);
    *batch = {};

    if (!page_.Ready())
    {
      return 0;
    }

    auto reader = page_.TelemetryReader();

    RingScan scan = RingScan::DATA;
    uint32_t next = 0;
    const uint32_t count =
        reader.Since(last_seen_, &scan, &next, batch->ring, TELEMETRY_SLOTS);
    last_seen_ = next;

    if (scan == RingScan::GAP)
    {
      // 整段被写者覆写：不产生采样，但要让订阅者知道中间断了一段。
      // The writer overwrote the whole range: emit no samples but tell subscribers
      // an interval went missing.
      batch->gap = 1;
      batch->head = next;
      return 0;
    }

    batch->count = count;
    batch->head = next;
    return count;
  }

  /**
   * @brief 写一帧视觉参考。Write one frame of visual reference.
   * @param found 是否有目标。Whether a target was found.
   * @param aim_x 归一化中心 x。Normalised centre x.
   * @param aim_y 归一化中心 y。Normalised centre y.
   * @return 写入后的 `seq`。The resulting `seq`.
   */
  uint32_t WriteAim(bool found, float aim_x, float aim_y)
  {
    return page_.Region().WriteAim(found, aim_x, aim_y);
  }

  /**
   * @brief 写一个参数下行命令（已过大核白名单过滤）。
   *        Write one parameter-downlink command (already whitelist-filtered on the
   *        big core).
   *
   * @param param_id C606 侧参数表索引。Parameter-table index on the C606 side.
   * @param value 参数值；≤2^24 的整数在 f32 上精确。Parameter value; integers up to
   *              2^24 are exact in f32.
   * @param cmd 命令字，缺省 `CMD_PARAM`。Command code, `CMD_PARAM` by default.
   * @return 写入后的 `seq`。The resulting `seq`.
   */
  uint32_t WriteParam(uint16_t param_id, float value,
                      uint8_t cmd = Region::CMD_PARAM)
  {
    RegionPayload payload = {};
    payload.cmd = cmd;
    payload.param_id = param_id;
    payload.value = value;
    return page_.Region().Write(payload);
  }

  /**
   * @brief 当前遥测发布索引（`head`）。Current telemetry publish index (`head`).
   */
  [[nodiscard]] uint32_t TelemetryHead() const
  {
    return page_.TelemetryReader().Head();
  }

  /**
   * @brief 本适配器已消费到的发布索引（相当于订阅者的 `last_seen`）。
   *        Publish index this adapter has consumed (the subscriber-side
   *        `last_seen`).
   */
  [[nodiscard]] uint32_t LastSeen() const { return last_seen_; }

  /**
   * @brief 页视图。The page view.
   */
  [[nodiscard]] SharedPage& Page() { return page_; }

 private:
  SharedPage page_;      ///< 已映射的页。Already-mapped page.
  LibXR::Topic topic_;   ///< 遥测 topic。Telemetry topic.
  uint32_t last_seen_ = 0;  ///< 已发布到的遥测索引。Telemetry index published up to.
  uint64_t last_drain_us_ = 0;  ///< 上次 drain 的时刻（us）。Time of the last drain (us).
  uint32_t period_us_ = 1000;   ///< drain 节律（us）。Drain cadence (us).
};

}  // namespace SharedPageXR
