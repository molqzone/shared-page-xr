#pragma once

#include <cstddef>
#include <cstdint>

#include "shared_page.hpp"
#include "topic.hpp"

/**
 * @file linux_shared_page.hpp
 * @brief 共享页在 Linux 侧的 Topic 适配（对齐 `LinuxSharedTopic`）。
 *        Linux-side topic adapter for a shared page, matching the `LinuxSharedTopic`
 *        idiom.
 *
 * 页只是 `SharedPage` 后端里的传输：模块在两侧看到的都是 topic，页知识只留在本文件。
 * C606 侧不需要它。
 * A page is only the transport behind `SharedPage`: modules see topics on both sides
 * and all page knowledge stays in this file. The C606 side does not need it.
 *
 * 遥测（C606 → Linux）由 `Poll()` 以配置节律 drain 成一条 `TelemetryBatch` 发到
 * `telemetry` topic；参考/命令（Linux → C606）由 `WriteAim()` 与 `WriteParam()` 写，
 * C606 控制环直接读 region。
 * Telemetry (C606 to Linux) is drained by `Poll()` at the configured cadence into one
 * `TelemetryBatch` on the `telemetry` topic; the reference and command direction
 * (Linux to C606) is written by `WriteAim()` and `WriteParam()`, which the C606
 * control loop reads straight out of the region.
 *
 * 模块拿到的是已经映射好的页，本适配器不碰 `/dev/mem`：非缓存映射需要 root，属平台
 * 初始化而不是模块行为。
 * The module receives an already-mapped page and this adapter never touches
 * `/dev/mem`: the non-cached mapping needs root and belongs to platform
 * initialisation rather than module behaviour.
 */

namespace LibXR
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
 * `head` 是本批之后的发布索引，订阅者据此保持自己的 `last_seen`；`gap` 表示更早的历史
 * 已被写者覆写，订阅者应记一段无效区间而不是补齐。`ring` 是值拷贝：回调返回后页可能
 * 已被覆写，载荷必须自持数据。
 * `head` is the publish index after this batch, which is how a subscriber keeps its own
 * `last_seen`; `gap` marks that older history was overwritten, so a subscriber records
 * an invalid interval instead of filling it in. The `ring` slots are copies: the page
 * may be overwritten after the callback returns, so the payload owns its data.
 */
struct TelemetryBatch
{
  Sample ring[TELEMETRY_SLOTS] = {};  ///< 区间内的采样，正序。Samples of this range, in
                                      ///< ascending order.
  uint32_t count = 0;        ///< 有效条数，≤ `TELEMETRY_SLOTS`。Valid sample count.
  uint32_t head = 0;         ///< 本批之后的发布索引。Publish index after this batch.
  uint8_t gap = 0;           ///< 1 = 本批之前有被覆写的区间。1 = earlier history was
                             ///< overwritten.
  uint8_t reserved[3] = {};  ///< 对齐留白。Alignment padding.
};

/// @brief `TelemetryBatch` 的字节数，topic payload 契约的一部分。Byte size of
///        `TelemetryBatch`, part of the topic payload contract.
inline constexpr size_t TELEMETRY_BATCH_BYTES = sizeof(TelemetryBatch);

static_assert(sizeof(TelemetryBatch) == sizeof(Sample) * TELEMETRY_SLOTS + 16,
              "TelemetryBatch layout pinned");

/**
 * @struct LinuxSharedPageConfig
 * @brief Linux 侧共享页适配器的创建配置。Creation config of the Linux-side adapter.
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
 *        Linux-side shared page: drain the telemetry range into a topic and write the
 *        reference region.
 *
 * 不拥有页内存，也不拥有 topic：两者都由初始化代码构造后注入，与 `LinuxSharedTopic`
 * 由调用者持有句柄一致。
 * It owns neither the page memory nor the topic: both are constructed by initialisation
 * code and injected here, matching how a `LinuxSharedTopic` handle is owned by its
 * caller.
 */
class LinuxSharedPage
{
 public:
  /**
   * @brief 绑定页与 topic。Bind the page and the topic.
   * @param page 已映射的页视图。Already-mapped page view.
   * @param topic 遥测 topic（payload 为 `TelemetryBatch`）。Telemetry topic whose payload
   *              is `TelemetryBatch`.
   * @param config 创建配置。Creation config.
   */
  LinuxSharedPage(SharedPage page, LibXR::Topic topic,
                  const LinuxSharedPageConfig& config = {});

  LinuxSharedPage(const LinuxSharedPage&) = delete;
  LinuxSharedPage& operator=(const LinuxSharedPage&) = delete;

  /**
   * @brief 页是否处在可用状态（已绑定且魔术字正确）。
   *        Whether the page is usable (bound and the magic matches).
   */
  [[nodiscard]] bool Ready() const;

  /**
   * @brief 参考/命令区视图（供写 aim 与参数）。Reference/command view (for writing the
   * aim and parameters).
   */
  [[nodiscard]] Reference Region();

  /**
   * @brief 按节律 drain 一次（正常路径每周期调用一次）。Drain once when the cadence is
   *        due, which the normal path calls every cycle.
   *
   * 未到节律时不发布。Nothing is published when the cadence is not due.
   *
   * @param now_us 当前时间（us）；缺省由内部读取，测试可注入。Current time in
   *               microseconds; read internally by default and injectable for tests.
   * @return 本次发布了遥测返回 `true`。`true` when a batch was published.
   */
  bool Poll(uint64_t now_us = UINT64_MAX);

  /**
   * @brief 立即 drain 并填充一组，不发布（测试与 `RazverMaster` 组 metadata 用）。
   *        Drain now into one batch without publishing, used by tests and by
   *        `RazverMaster` when it assembles metadata.
   *
   * @param batch 输出：本组遥测。Output: this telemetry batch.
   * @return 本组采样条数，0 表示无新数据。Sample count of this batch; 0 means no new
   * data.
   */
  uint32_t Drain(TelemetryBatch* batch);

  /**
   * @brief 写一帧视觉参考。Write one frame of visual reference.
   * @param found 是否有目标。Whether a target was found.
   * @param aim_x 归一化中心 x。Normalised centre x.
   * @param aim_y 归一化中心 y。Normalised centre y.
   * @return 写入后的 `seq`。The resulting `seq`.
   */
  uint32_t WriteAim(bool found, float aim_x, float aim_y);

  /**
   * @brief 写一个参数下行命令（已过大核白名单过滤）。
   *        Write one parameter-downlink command (already whitelist-filtered on the big
   *        core).
   *
   * @param param_id C606 侧参数表索引。Parameter-table index on the C606 side.
   * @param value 参数值；≤2^24 的整数在 f32 上精确。Parameter value; integers up to 2^24
   *              are exact in f32.
   * @param cmd 命令字，缺省 `CMD_PARAM`。Command code, `CMD_PARAM` by default.
   * @return 写入后的 `seq`。The resulting `seq`.
   */
  uint32_t WriteParam(uint16_t param_id, float value, uint8_t cmd = Region::CMD_PARAM);

  /**
   * @brief 本适配器已消费到的发布索引（订阅者的 `last_seen`）。Publish index this adapter
   *        has consumed, the subscriber-side `last_seen`.
   */
  [[nodiscard]] uint32_t LastSeen() const;

 private:
  SharedPage page_;             ///< 已映射的页。Already-mapped page.
  LibXR::Topic topic_;          ///< 遥测 topic。Telemetry topic.
  uint32_t last_seen_ = 0;      ///< 已发布到的遥测索引。Telemetry index published up to.
  uint64_t last_drain_us_ = 0;  ///< 上次 drain 的时刻（us）。Time of the last drain (us).
  uint32_t period_us_ = 1000;   ///< drain 节律（us）。Drain cadence (us).
};

}  // namespace LibXR
