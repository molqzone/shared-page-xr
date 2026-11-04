#pragma once

#include <cstddef>
#include <cstdint>

#include "doorbell.hpp"
#include "shared_page.hpp"

/**
 * @file command_channel.hpp
 * @brief 命令方向：等通知、读区域、校验、回执 / Command direction: wait, read,
 *        validate, reply.
 *
 * 命令方向的跨核内容全部在这里：页是否可用、区域快照是否自洽、载荷是不是一条有效
 * 命令、重复的通知要不要重放、失败怎么回执。向上只剩两条调用：
 *
 * ```cpp
 * guidance::Command command = {};
 * uint32_t seq = 0;
 * if (channel.Wait(UINT32_MAX, &command, &seq) == LibXR::ErrorCode::OK) {
 *   channel.Ack(seq, ApplyCommand(command));   // 产品只写这一行业务
 * }
 * ```
 *
 * 通知侧是注入的 `Doorbell`：mailbox context 命令、rpmsg endpoint 或别的，页层不认识。
 * 回执也归它，因为“回给刚收到的那一条”是发送方协议，不是产品逻辑。
 *
 * Everything cross-core about the command direction lives here: whether the page is
 * usable, whether the region snapshot is coherent, whether the payload is a valid
 * command, whether a repeated notification should replay, and how a failure is
 * answered. Only two calls remain above:
 *
 * ```cpp
 * guidance::Command command = {};
 * uint32_t seq = 0;
 * if (channel.Wait(UINT32_MAX, &command, &seq) == LibXR::ErrorCode::OK) {
 *   channel.Ack(seq, ApplyCommand(command));   // the product writes only this
 * }
 * ```
 *
 * The notification side is an injected `Doorbell` -- a mailbox context command, an rpmsg
 * endpoint, anything -- and the reply lives there too, because "answer the one just
 * received" is the sender's protocol, not product logic.
 */

namespace LibXR
{
/** @brief 命令被拒绝时回给发送方的字 / The word reported back for a rejected command.
 *
 * 与 `ApplyCommand` 的拒绝返回值同源：发送方只需要区分“被接受（回显 seq）”和
 * “被拒绝（这个字）”。
 * Same source as `ApplyCommand`'s reject value: the sender only distinguishes
 * "accepted (seq echoed)" from "rejected (this word)".
 */
inline constexpr uint32_t COMMAND_REJECTED = UINT32_MAX;

/**
 * @brief 一条命令通道：把页的区域读成“已校验、去重的命令” / One command channel:
 *        turn the page's region into a validated, de-duplicated command.
 *
 * 模板参数与 `SharedPage` 一致，因为区域偏移由 `F`/`P` 决定、页头互验还要 `T` 与
 * `TAG`。通道持有页的副本（与视图同风格：拷贝即同一映射的另一个视图），所以调用方
 * 不必保证页对象的生命周期比通道长。
 *
 * 去重：同一区域 `seq` 的重复通知（doorbell 合并，或发送方没收到回执而重试）不重放
 * 命令，只重发上一次的回执——否则发送方会超时，而重放会把 LED/PARK 这类命令执行
 * 第二次。
 *
 * Same template parameters as `SharedPage`, because the region offset follows `F`/`P`
 * and the header cross-check also needs `T` and `TAG`. The channel keeps a copy of the
 * page (same style as the views: a copy is another view of the same mapping), so the
 * caller does not have to keep the page object alive.
 *
 * Dedup: a repeated notification for the same region `seq` (coalesced doorbells, or a
 * sender retrying because it missed the reply) does not replay the command; it resends
 * the previous reply. Otherwise the sender times out, and replaying would run an
 * LED/PARK command twice.
 */
template <typename F, PagePod T, PagePod P, uint32_t TAG = 0,
          CoherencePolicyType Policy = FunctionPointerPolicy>
class CommandChannel
{
 public:
  using Page = SharedPage<F, T, P, TAG, Policy>;

  /// @brief 应用侧的载荷校验：magic/类型等，由产品给。
  ///        Application-side payload check (magic, type, ...), supplied by the product.
  using ValidateFn = bool (*)(const P& payload);

  CommandChannel(const Page& page, Doorbell doorbell, ValidateFn validate)
      : page_(page), doorbell_(doorbell), validate_(validate)
  {
  }

  /**
   * @brief 等一条通过校验的命令。
   * @param timeout_ms 通知等待上限；`UINT32_MAX` 表示一直等。
   * @param out 收到的命令；只在返回 `OK` 时写入。
   * @param seq 该命令的区域序号；只在返回 `OK` 时写入，回执时原样交回 `Ack`。
   * @return `OK` 有命令；`TIMEOUT` 没有通知；`FAILED` 页不可用（已回执拒绝）；
   *         `EMPTY` 命令无效、或是重复通知（均已回执）；`BUSY` 区域快照不收敛
   *         （已回执拒绝）；`PTR_NULL` 参数或页为空。
   *
   * Wait for one validated command. `out` and `seq` are written only on `OK`.
   */
  [[nodiscard]] ErrorCode Wait(uint32_t timeout_ms, P* out, uint32_t* seq)
  {
    if (out == nullptr || seq == nullptr || !page_.Valid())
    {
      return ErrorCode::PTR_NULL;
    }

    const ErrorCode notified = doorbell_.Wait(timeout_ms);
    if (notified != ErrorCode::OK)
    {
      // 没有在途命令：不回执，否则会回答一条不存在的命令。
      // Nothing outstanding: no reply, or we would answer a command that is not there.
      return notified;
    }

    if (!page_.Ready())
    {
      // Linux 负责格式化；没有页头就没有命令槽可读。
      // Linux owns formatting; without a header there is no slot to read.
      doorbell_.Reply(COMMAND_REJECTED);
      return ErrorCode::FAILED;
    }

    P payload = {};
    uint32_t observed = 0;
    const ErrorCode read = page_.Region().Read(&payload, &observed);
    if (read != ErrorCode::OK)
    {
      doorbell_.Reply(COMMAND_REJECTED);
      return read;
    }

    if (validate_ == nullptr || !validate_(payload))
    {
      doorbell_.Reply(COMMAND_REJECTED);
      return ErrorCode::EMPTY;
    }

    if (have_last_ && observed == last_seq_)
    {
      doorbell_.Reply(last_result_);
      return ErrorCode::EMPTY;
    }

    *out = payload;
    *seq = observed;
    last_seq_ = observed;
    have_last_ = true;
    return ErrorCode::OK;
  }

  /**
   * @brief 回答最近一次 `Wait()` 收到的命令。
   * @param seq `Wait()` 交回的序号；与在途命令不符时只记断言，不发送。
   * @param result 回给发送方的字（通常是被接受命令的 seq，拒绝用
   *        `COMMAND_REJECTED`）。
   *
   * Answer the command of the most recent `Wait()`. A seq that does not match the
   * outstanding command trips an assertion and is not sent.
   */
  void Ack(uint32_t seq, uint32_t result)
  {
    ASSERT(!have_last_ || seq == last_seq_);
    (void)seq;
    last_result_ = result;
    doorbell_.Reply(result);
  }

 private:
  Page page_;
  Doorbell doorbell_;
  ValidateFn validate_;
  uint32_t last_seq_ = 0;
  uint32_t last_result_ = 0;
  bool have_last_ = false;
};

}  // namespace LibXR
