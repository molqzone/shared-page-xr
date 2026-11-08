#pragma once

#include <cstdint>

#include "libxr_def.hpp"

/**
 * @file doorbell.hpp
 * @brief 页层的“响铃”抽象：这一侧对另一个核的全部通知依赖 /
 *        The page layer's ringing abstraction: everything this side needs in order to
 *        notify the other core.
 *
 * 页引擎（`shared_page.hpp`）只搬字节；*什么时候*让另一个核知道有新字节，是门铃的
 * 事。门铃可以是 mailbox context 里的 8 字节命令、一个 rpmsg endpoint、一块 UIO 的
 * `read()`、一条外部中断线——页层不认识，也不该认识。
 *
 * The page engine (`shared_page.hpp`) only moves bytes.  *When* the other core learns
 * that there are new bytes is the doorbell's business.  A doorbell can be an 8-byte
 * command in a mailbox context, an rpmsg endpoint, a UIO `read()`, an external
 * interrupt line -- the page layer neither knows nor should know.
 *
 * 三个动词覆盖了两个方向，缺一不可 / Three verbs cover both directions; none is
 * optional:
 *
 *  - `Wait()`：被叫醒。`OK` 表示**确有一条在途请求**，调用方必须用 `Reply()` 回答它；
 *    其它返回值表示没有在途请求，`Reply()` 不得调用。
 *  - `Reply()`：回答最近一次成功 `Wait()` 的那条请求；只在门铃本身承载回执时存在
 *    （mailbox 的 context 命令就是这样）。
 *  - `Signal()`：无载荷地叫醒对端——数据的字节已经在页里，铃声只说明“可以读了”。
 *
 *  - `Wait()`: be woken.  `OK` means exactly one request is outstanding and the caller
 *    must answer it with `Reply()`; any other result means nothing is outstanding and
 *    `Reply()` must not be called.
 *  - `Reply()`: answer the request of the most recent successful `Wait()`.  It exists
 *    because the doorbell itself can carry the answer -- a mailbox context command
 *    does.
 *  - `Signal()`: wake the peer without a payload -- the bytes are already in the page,
 *    and the ring only says "you may read".
 *
 * ## 排序合同 / The ordering contract
 *
 * `Signal()` 是**发布边沿**：在它之前写进页的字节，必须在对端的 `Wait()` 返回之后可见。
 * 实现要在 `Signal()` 里给出这一保证（软件挂起位用 release/acquire，硬件门铃用“中断不
 * 得先于共享内存写入对端可见”），而不是让调用方自己 fence 两次。
 * `Signal()` is the publication edge: bytes written to the page before it must be
 * visible after the peer's `Wait()` returns.  The implementation owes that guarantee --
 * a release/acquire pair for software pending bits, "the interrupt must not be observed
 * before the shared-memory writes" for a hardware doorbell -- instead of making callers
 * fence on both sides themselves.
 *
 * ## 为什么 `Wait()` 不报告“哪个通道响了” / Why `Wait()` reports no channel
 *
 * 本产品的门铃只有一条命令通道：Linux 只往命令上下文中放请求，通知位只由固件敲。没有
 * 多路，就没有“哪一个响了”这个问题——加一个返回值只会成为一个恒为 0 的参数。等到真的
 * 一铃多路（例如同一门铃同时承载命令与 maint 请求），这里改成
 * `Wait(timeout, uint32_t* channel)` 并在合同里写清每个通道的归属即可，调用方那两处
 * 加一个实参，不需要重排分层。
 * This product's doorbell has exactly one command channel: Linux only places requests in
 * command contexts, and the notification bit is only ever rung by the firmware.  With no
 * multiplexing there is no "which one fired" question -- a return value would be a
 * constant parameter.  If a second channel ever shares the doorbell (commands plus
 * maint, say), the signature grows to `Wait(timeout, uint32_t* channel)` and the
 * contract gains a channel map; the two call sites pass one more argument, and no layer
 * moves.
 *
 * 为什么是函数指针：这个仓是**中间件**，不是驱动层级。LibXR 里两种接缝分工明确——
 * 平台驱动往上实现虚基类（`src/driver/` 的 `UART`/`SPI`/`GPIO`），中间件从外部注入
 * 函数指针（`ReadPort::read_fun_`、本仓的 `Coherent` provider、`ValidateFn`）。页层属于
 * 后者：它不认识设备，只要求“被叫醒、回答、叫醒”，所以用注入的函数指针，而不是让页层
 * 继承一个驱动基类。
 *
 * 也不用 `LibXR::Callback`：那是订阅列表用的（`Topic`/`Event`）——内部 `new` 一个
 * 回调块、返回 void、带 `in_isr` 标志；而这条接缝需要返回值（`Wait` 的结果）、在堆
 * 还没起来时就要可用。
 *
 * Why function pointers: this repository is **middleware**, not the driver hierarchy.
 * LibXR splits the two seams cleanly -- platform drivers implement virtual bases
 * (`UART`/`SPI`/`GPIO` under `src/driver/`), middleware takes function pointers
 * injected from outside (`ReadPort::read_fun_`, this repo's `Coherent` provider and
 * `ValidateFn`).  The page layer is the latter: it knows no device, it only asks to be
 * woken, to answer, and to wake -- so injected function pointers, not a driver base
 * class it would have to inherit.
 *
 * `LibXR::Callback` is not used either: that one serves subscriber lists (`Topic`/`Event`)
 * -- it `new`s a callback block, returns void and carries an `in_isr` flag -- while this
 * seam must return a value (`Wait`'s result) and be usable before the heap is up.
 */
namespace LibXR
{
/** @brief 页层的门铃接缝 / The page layer's doorbell seam. */
class Doorbell
{
 public:
  /// @brief 等一条在途请求；`OK` 即“有一条要回答”。
  ///        Wait for an outstanding request; `OK` means one is waiting for an answer.
  using WaitFn = ErrorCode (*)(void* context, uint32_t timeout_ms);
  /// @brief 无载荷叫醒对端。Wake the peer without a payload.
  using SignalFn = void (*)(void* context);
  /// @brief 回答最近一条在途请求。Answer the most recent outstanding request.
  using ReplyFn = void (*)(void* context, uint32_t result);

  constexpr Doorbell(WaitFn wait, SignalFn signal, ReplyFn reply, void* context)
      : wait_(wait), signal_(signal), reply_(reply), context_(context)
  {
  }

  /// @brief 等一条通知；`OK` 即“有在途请求，必须 `Reply()`”。
  ///        Wait for one notification; `OK` means one request is outstanding.
  [[nodiscard]] ErrorCode Wait(uint32_t timeout_ms) const
  {
    return wait_(context_, timeout_ms);
  }

  /// @brief 无载荷叫醒对端（数据在页里）。Wake the peer; the data is in the page.
  void Signal() const { signal_(context_); }

  /// @brief 回答最近一条在途请求。Answer the most recent outstanding request.
  void Reply(uint32_t result) const { reply_(context_, result); }

 private:
  WaitFn wait_;
  SignalFn signal_;
  ReplyFn reply_;
  void* context_;
};
}  // namespace LibXR
