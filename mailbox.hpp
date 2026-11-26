#pragma once

#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR
{

class Mailbox;

/// @brief RX 回调：裸函数指针加 user 数据，因为这条路径要在堆起来之前就能注册，
///        而 `LibXR::Callback` 内部分配。
///        RX callback: a raw function pointer plus user data, because this path
///        must be registrable before the heap is up and `LibXR::Callback`
///        allocates.
using MailboxRxCallback = void (*)(void* user);

/**
 * @brief 核间通知外设 / The inter-core notification peripheral.
 *
 * 这颗外设在每颗 AMP SoC 上都有，只是名字不同：WCH CH32H417 和 STM32H7 叫 HSEM
 * （32 颗信号量，take/release 触发对端中断），STM32WB/MP1 叫 IPCC（12 通道 × 2
 * 方向），SG2002 用 mailbox。共同形状是**一个控制器、N 个通道、按通道通知**。
 *
 * Every AMP SoC has this peripheral under a different name: HSEM on WCH CH32H417
 * and STM32H7 (32 semaphores, take/release raises the peer's interrupt), IPCC on
 * STM32WB/MP1 (12 channels per direction), a mailbox on SG2002. The common shape
 * is one controller, N channels, notification per channel.
 *
 * 与 Zephyr 的 MBOX 对应：`mbox_send` → `Signal`、`mbox_register_callback` →
 * `RegisterCallback`、`mbox_set_enabled` → `SetEnabled`，devicetree 的
 * `mboxes = <&ipcc 1 TX>` 通道绑定对应板子调 `GetChannel()` 的那一下。
 *
 * Mapping to Zephyr's MBOX: `mbox_send` → `Signal`, `mbox_register_callback` →
 * `RegisterCallback`, `mbox_set_enabled` → `SetEnabled`, and devicetree's
 * `mboxes = <&ipcc 1 TX>` channel binding corresponds to the board's
 * `GetChannel()` call.
 *
 * Zephyr 的 MBOX 有 MTU 和载荷，这里**故意没有**：本仓的全部设计是"字节住在页里，
 * 门铃只说明可以读了"。页有 `Coherent` provider 管可见性、`head`/`seq` 的
 * acquire-release 管撕裂、`Check()` 互验几何。门铃一旦能带载荷，就会有人往里塞没
 * 经过这些检查的数据——那正是本仓删掉宿主适配器的原因（"会把未校验的样本 publish
 * 出去"）。需要载荷的板子应该扩页协议，而不是在通知边上开第二条数据路径。
 *
 * Zephyr's MBOX has an MTU and a payload; this one deliberately does not: this
 * repository's whole design is "the bytes live in the page, the doorbell only
 * says they can be read". The page has a `Coherent` provider for visibility,
 * acquire-release on `head`/`seq` against tearing, and `Check()` for geometry
 * drift. A payload-carrying doorbell invites data that bypasses all three --
 * which is exactly why this repository deleted its host adapter ("would publish
 * unvalidated samples"). A board that needs a payload should extend the page
 * protocol, not open a second data path on the notification edge.
 *
 * ## 为什么是函数指针，不是驱动基类 / Why function pointers, not a driver base
 *
 * 本仓是中间件，不是驱动层级：平台驱动往上实现虚基类（libxr 的 `src/driver/`），
 * 中间件从外部注入函数指针。`Mailbox` 依旧不认识任何具体外设——它只要求"按通道
 * 叫醒、按通道等、按通道回答、按通道开关"。`Mailbox::Channel` 是这层注入之上的
 * 薄薄一层类型化外壳，不引入虚函数，也不要求外设继承什么。
 *
 * This repository is middleware, not the driver hierarchy: platform drivers
 * implement virtual bases (libxr's `src/driver/`), middleware takes injected
 * function pointers. `Mailbox` still knows no concrete peripheral -- it only asks
 * to wake, wait, reply and enable, per channel. `Mailbox::Channel` is a thin typed
 * shell over that injection: no virtual functions, nothing to inherit.
 *
 * ## 对实现者的要求 / Note for implementers
 *
 * 背后是块级中断外设时，中断向量必须由一个驱动统一持有：每条通道在 `GetChannel()`
 * 时绑定，ISR 遍历它们。不要让每个消费者各写一个 handler——符号表只会选中一个，
 * 另一个静默失效，而它的挂起位没人清，最终是中断风暴。libxr 的
 * `driver/ch/ch32_hsem.*` 是这份要求的参考实现。
 *
 * When the peripheral is block-interrupt based, one driver must own the vector:
 * channels bind in `GetChannel()` and the ISR walks them. Never let each
 * consumer write its own handler -- the linker keeps one and the other silently
 * never runs, leaving its pending bit set, which ends in an interrupt storm. See
 * libxr's `driver/ch/ch32_hsem.*` for a reference implementation.
 */
class Mailbox
{
 public:
  /**
   * @brief 本外设的一条通道 / One channel of this peripheral.
   *
   * 形状照 libxr 的 `STM32ADC::Channel`：外设发句柄，句柄上带操作，句柄知道自己
   * 属于哪颗外设、是哪一条通道。定义在类外（本头文件后半部分），因为它的每个动作
   * 都要转发给所属外设，那时外设必须已经完整。
   *
   * The shape follows libxr's `STM32ADC::Channel`: the peripheral hands out a
   * handle, the handle carries the operations, and it knows which peripheral it
   * belongs to and which channel it is. Defined outside the class (later in this
   * header) because every handle action forwards to its peripheral, which must be
   * complete by then.
   */
  class Channel;

  /// @brief RX 回调：裸函数指针加 user 数据，因为这条路径要在堆起来之前就能注册，
  ///        而 `LibXR::Callback` 内部分配。
  ///        RX callback: a raw function pointer plus user data, because this path
  ///        must be registrable before the heap is up and `LibXR::Callback`
  ///        allocates.
  using RxCallback = void (*)(void* user);

  /// @brief 等任意一条通道的通知；`OK` 时 `*channel` 是响的那条。
  ///        Wait for a notification on any channel; on `OK` `*channel` names it.
  using WaitFn = ErrorCode (*)(void* context, uint32_t timeout_ms, uint32_t* channel);
  /// @brief 在 channel 上无载荷叫醒对端。Wake the peer on `channel`, no payload.
  using SignalFn = void (*)(void* context, uint32_t channel);
  /// @brief 回答 channel 上最近一次 Wait 等到的那条请求。
  ///        Answer the request the most recent Wait saw on `channel`.
  using ReplyFn = void (*)(void* context, uint32_t channel, uint32_t result);
  /// @brief 开/关一条通道的中断；平台没有中断可关时给空实现。
  ///        Enable or disable one channel's interrupt; an empty function where the
  ///        platform has none to disable.
  using SetEnabledFn = void (*)(void* context, uint32_t channel, bool enable);
  /// @brief 注册 RX 回调；平台没有回调能力时传 nullptr。
  ///        Register an RX callback; nullptr where the platform has no callback
  ///        capability.
  using RegisterCallbackFn = void (*)(void* context, uint32_t channel,
                                      RxCallback callback, void* user);

  /**
   * @param wait               等通知，见 WaitFn
   * @param signal             叫醒对端，见 SignalFn
   * @param reply              回执，见 ReplyFn
   * @param set_enabled        通道开关，见 SetEnabledFn；可为 nullptr
   * @param register_callback  RX 回调，见 RegisterCallbackFn；可为 nullptr
   * @param channels           通道句柄存储，由板子提供，长度见 channel_count
   * @param channel_count      通道条数，`GetChannel()` 的边界
   * @param context            透传给所有回调的板级上下文
   */
  constexpr Mailbox(WaitFn wait, SignalFn signal, ReplyFn reply,
                    SetEnabledFn set_enabled, RegisterCallbackFn register_callback,
                    Channel* channels, uint32_t channel_count, void* context)
      : wait_(wait),
        signal_(signal),
        reply_(reply),
        set_enabled_(set_enabled),
        register_callback_(register_callback),
        channels_(channels),
        channel_count_(channel_count),
        context_(context)
  {
  }

  /**
   * @brief 取一条通道 / Get one channel
   * @param id 通道号，必须小于构造时的 channel_count
   * @return const Channel& 句柄，存储在板子给的数组里
   *
   * 越界时断言失败——通道号是板子在编译期就知道的常量，传错是代码错误，不是运行期
   * 状况。
   * Out of range asserts: a channel number is a compile-time constant on the
   * board, so a wrong one is a code error, not a runtime condition.
   */
  /// @brief 取一条通道；定义在 `Channel` 之后，因为它要绑定句柄。
  ///        Get one channel; defined after `Channel` because it binds a handle.
  [[nodiscard]] const Channel& GetChannel(uint32_t id) const;

  /// @brief 本外设的通道条数。How many channels this peripheral has.
  [[nodiscard]] uint32_t ChannelCount() const { return channel_count_; }

  /// @brief 等一条通知；`OK` 即"channel 上有一条在途请求，必须 Reply()"。
  ///        Wait for one notification; `OK` means one request is outstanding on
  ///        `channel` and must be answered with Reply().
  [[nodiscard]] ErrorCode Wait(uint32_t timeout_ms, uint32_t* channel) const
  {
    return wait_(context_, timeout_ms, channel);
  }

  /// @brief 在 channel 上叫醒对端。Wake the peer on `channel`.
  void Signal(uint32_t channel) const { signal_(context_, channel); }

  /// @brief 回答 channel 上最近一条在途请求。Answer the most recent outstanding
  ///        request on `channel`.
  void Reply(uint32_t channel, uint32_t result) const
  {
    reply_(context_, channel, result);
  }

  /// @brief 开/关一条通道的中断；驱动没给这个能力时是空操作。
  ///        Enable or disable one channel's interrupt; a no-op when the driver
  ///        offered no such capability.
  void SetEnabled(uint32_t channel, bool enable) const
  {
    if (set_enabled_ != nullptr)
    {
      set_enabled_(context_, channel, enable);
    }
  }

  /// @brief 注册 RX 回调；驱动没给这个能力时是空操作。
  ///        Register an RX callback; a no-op when the driver offered no such
  ///        capability.
  void RegisterCallback(uint32_t channel, RxCallback callback, void* user) const
  {
    if (register_callback_ != nullptr)
    {
      register_callback_(context_, channel, callback, user);
    }
  }

 private:
  WaitFn wait_;
  SignalFn signal_;
  ReplyFn reply_;
  SetEnabledFn set_enabled_;
  RegisterCallbackFn register_callback_;
  Channel* channels_;
  uint32_t channel_count_;
  void* context_;
};

// ---- Mailbox::Channel ------------------------------------------------------
// 定义在 Mailbox 之后：句柄的每个动作都转发给所属外设，那时外设必须已经完整。
// Defined after Mailbox: every handle action forwards to its peripheral, which must
// be complete by then.

class Mailbox::Channel
{
 public:
  Channel() = default;

  /**
   * @brief 等一条通知；`OK` 即"本通道上有一条在途请求，必须 Reply()"。
   *        Wait for one notification; `OK` means one request is outstanding on
   *        this channel and must be answered with Reply().
   *
   * 只等本通道：别的通道响了会继续等，不会误当成本通道的命令——对命令方向来说那就是
   * "没有在途命令"，而且绝不能回执。
   * Waits for this channel only: another channel ringing keeps waiting rather than
   * being mistaken for a command on this one. For the command direction that is
   * "nothing outstanding", and it must not be answered.
   */
  [[nodiscard]] ErrorCode Wait(uint32_t timeout_ms) const;

  /**
   * @brief 无载荷叫醒对端（发布边沿）。
   *        Wake the peer, no payload (the publication edge).
   */
  void Signal() const;

  /**
   * @brief 回答本通道上最近一次 `Wait()` 等到的那条请求。
   *        Answer the request the most recent `Wait()` saw on this channel.
   */
  void Reply(uint32_t result) const;

  /// @brief 开/关本通道的中断；外设没这个能力时是空操作。
  ///        Enable or disable this channel's interrupt; a no-op when the
  ///        peripheral offered no such capability.
  void SetEnabled(bool enable) const;

  /// @brief 注册本通道的 RX 回调（推送式消费）；外设没这个能力时是空操作。
  ///        Register this channel's RX callback (push-style consumption); a
  ///        no-op when the peripheral offered no such capability.
  void RegisterCallback(MailboxRxCallback callback, void* user) const;

  /// @brief 本通道在所属外设里的编号。This channel's number in its peripheral.
  [[nodiscard]] uint32_t Id() const { return id_; }

 private:
  void Bind(const Mailbox* mailbox, uint32_t id)
  {
    mailbox_ = mailbox;
    id_ = id;
  }

  const Mailbox* mailbox_ = nullptr;
  uint32_t id_ = 0;

  friend class Mailbox;
};

inline ErrorCode Mailbox::Channel::Wait(uint32_t timeout_ms) const
{
  uint32_t channel = 0;
  const ErrorCode result = mailbox_->Wait(timeout_ms, &channel);
  if (result != ErrorCode::OK)
  {
    return result;
  }
  return channel == id_ ? ErrorCode::OK : ErrorCode::TIMEOUT;
}

inline void Mailbox::Channel::Signal() const { mailbox_->Signal(id_); }

inline void Mailbox::Channel::Reply(uint32_t result) const
{
  mailbox_->Reply(id_, result);
}

inline void Mailbox::Channel::SetEnabled(bool enable) const
{
  mailbox_->SetEnabled(id_, enable);
}

inline void Mailbox::Channel::RegisterCallback(MailboxRxCallback callback,
                                               void* user) const
{
  mailbox_->RegisterCallback(id_, callback, user);
}

inline const Mailbox::Channel& Mailbox::GetChannel(uint32_t id) const
{
  ASSERT(id < channel_count_);
  channels_[id].Bind(this, id);
  return channels_[id];
}

}  // namespace LibXR
