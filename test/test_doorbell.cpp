/**
 * @file test_doorbell.cpp
 * @brief `Doorbell` 接缝的三条转发契约 / The three forwarding contracts of the
 *        `Doorbell` seam.
 *
 * 这层薄到一眼能看完，正因为它薄，写错的代价是整条链路：三个同类型的函数指针按位置
 * 传参，谁先谁后只有编译期签名挡得住。所以这里逐条钉住“哪个动词走到哪个指针、带什么
 * 参数、不碰别的指针”，并确认 `Signal()` 与 `Reply()` 互不越界——门铃的“响铃”和
 * “回执”在 mailbox 上是同一套寄存器，混淆了就会在板上表现为“遥测把回执吃掉”。
 *
 * The seam is thin enough to read in one go, and because it is thin a mistake costs the
 * whole link: three same-typed function pointers are passed positionally, so only the
 * signature stops a mix-up.  These tests pin which verb reaches which pointer, with
 * which argument, and that it leaves the others alone -- `Signal()` and `Reply()` share
 * one set of mailbox registers on this platform, so confusing them would show up on the
 * board as "telemetry ate the reply".
 */

#include <cstdint>

#include "doorbell.hpp"
#include "test_assert.hpp"

namespace
{
using LibXR::Doorbell;
using LibXR::ErrorCode;

/// @brief 记录调用顺序与参数的门铃实现 / A recorder: call order and arguments.
struct Recorder
{
  unsigned waits = 0;
  unsigned signals = 0;
  unsigned replies = 0;
  uint32_t last_timeout = 0;
  uint32_t last_result = 0;
  ErrorCode wait_result = ErrorCode::OK;
};

ErrorCode RecordWait(void* context, uint32_t timeout_ms)
{
  auto* recorder = static_cast<Recorder*>(context);
  ++recorder->waits;
  recorder->last_timeout = timeout_ms;
  return recorder->wait_result;
}

void RecordSignal(void* context)
{
  auto* recorder = static_cast<Recorder*>(context);
  ++recorder->signals;
}

void RecordReply(void* context, uint32_t result)
{
  auto* recorder = static_cast<Recorder*>(context);
  ++recorder->replies;
  recorder->last_result = result;
}

/// 1. `Wait()` 把超时原样交给实现，并原样带回结果。
///    1. `Wait()` hands the timeout through and returns the implementation's result.
void test_wait_forwards_timeout()
{
  Recorder recorder;
  const Doorbell doorbell{&RecordWait, &RecordSignal, &RecordReply, &recorder};

  TEST_ASSERT(doorbell.Wait(0) == ErrorCode::OK);
  TEST_ASSERT(recorder.waits == 1);
  TEST_ASSERT(recorder.last_timeout == 0);
  TEST_ASSERT(recorder.signals == 0);
  TEST_ASSERT(recorder.replies == 0);

  recorder.wait_result = ErrorCode::TIMEOUT;
  TEST_ASSERT(doorbell.Wait(UINT32_MAX) == ErrorCode::TIMEOUT);
  TEST_ASSERT(recorder.waits == 2);
  TEST_ASSERT(recorder.last_timeout == UINT32_MAX);
}

/// 2. `Signal()` 只响铃：不等待、不回执。
///    2. `Signal()` only rings: no wait, no reply.
void test_signal_does_not_reply()
{
  Recorder recorder;
  const Doorbell doorbell{&RecordWait, &RecordSignal, &RecordReply, &recorder};

  doorbell.Signal();
  TEST_ASSERT(recorder.signals == 1);
  TEST_ASSERT(recorder.waits == 0);
  TEST_ASSERT(recorder.replies == 0);

  doorbell.Signal();
  TEST_ASSERT(recorder.signals == 2);
  TEST_ASSERT(recorder.replies == 0);
}

/// 3. `Reply()` 只回执：把结果字原样交给实现，不响铃。
///    3. `Reply()` only answers: the result word reaches the implementation untouched,
///    and nothing rings.
void test_reply_does_not_signal()
{
  Recorder recorder;
  const Doorbell doorbell{&RecordWait, &RecordSignal, &RecordReply, &recorder};

  doorbell.Reply(0u);
  TEST_ASSERT(recorder.replies == 1);
  TEST_ASSERT(recorder.last_result == 0u);
  TEST_ASSERT(recorder.signals == 0);
  TEST_ASSERT(recorder.waits == 0);

  doorbell.Reply(UINT32_MAX);
  TEST_ASSERT(recorder.replies == 2);
  TEST_ASSERT(recorder.last_result == UINT32_MAX);
  TEST_ASSERT(recorder.signals == 0);
}

/// 4. 拷贝即同一门铃：视图可拷贝（页层的视图风格），拷贝后仍指向同一实现。
///    4. A copy is the same doorbell: views are copyable (the page layer's style) and a
///    copy still reaches the same implementation.
void test_copy_shares_the_binding()
{
  Recorder recorder;
  const Doorbell doorbell{&RecordWait, &RecordSignal, &RecordReply, &recorder};
  const Doorbell copy = doorbell;

  copy.Signal();
  copy.Reply(0x5Au);
  TEST_ASSERT(recorder.signals == 1);
  TEST_ASSERT(recorder.replies == 1);
  TEST_ASSERT(recorder.last_result == 0x5Au);
  TEST_ASSERT(doorbell.Wait(0) == ErrorCode::OK);
  TEST_ASSERT(recorder.waits == 1);
}
}  // namespace

int main()
{
  test_wait_forwards_timeout();
  test_signal_does_not_reply();
  test_reply_does_not_signal();
  test_copy_shares_the_binding();
  return 0;
}
