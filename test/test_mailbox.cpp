/**
 * @file test_mailbox.cpp
 * @brief `mailbox.hpp` 的契约测试 / Contract tests for `mailbox.hpp`.
 *
 * 测什么 / Checks:
 *   1. 五个动词各走各的指针——`Signal`/`Reply`/`SetEnabled`/`RegisterCallback` 在真实
 *      硬件上共用一套寄存器（HSEM 的 RLR/IPCC 的通道），混淆了在板上会表现成"遥测把
 *      回执吃掉"，所以在主机上先钉住。
 *   2. `Wait()` 报告是哪条通道响的——多通道是这个接缝存在的理由，通道号丢了就等于
 *      退回"一个通道一个 Mailbox"之前的形状。
 *   3. `SetEnabled` / `RegisterCallback` 传 nullptr 时是空操作，不崩。
 *   4. 超时原样透传给 WaitFn。
 *
 * Checks:
 *   1. Each verb reaches its own pointer -- they share one register set on real
 *      hardware (HSEM's RLR, IPCC's channels), so mixing them up would show up on
 *      the board as "telemetry ate the reply". Pinned on a host first.
 *   2. `Wait()` reports which channel rang -- channels are the reason this seam
 *      exists; losing the number is falling back to one channel.
 *   3. `SetEnabled` / `RegisterCallback` are no-ops when null.
 *   4. The timeout reaches WaitFn unchanged.
 *
 * 运行前提 / Setup: 无。全是注入的函数指针，不需要设备。
 * None. Everything is injected function pointers; no device needed.
 */

#include <cstdint>

#include "mailbox.hpp"
#include "test_assert.hpp"

using namespace LibXR;

namespace
{

struct Recorder
{
  int waits = 0;
  int signals[4] = {0, 0, 0, 0};
  int replies[4] = {0, 0, 0, 0};
  int enables[4] = {0, 0, 0, 0};
  int registers[4] = {0, 0, 0, 0};
  int callbacks = 0;
  uint32_t last_timeout = 0;
  uint32_t last_channel = 0xFFFFFFFFu;
};

Recorder rec;

ErrorCode RecWait(void*, uint32_t timeout_ms, uint32_t* channel)
{
  rec.waits++;
  rec.last_timeout = timeout_ms;
  if (channel != nullptr)
  {
    *channel = rec.last_channel;
  }
  return ErrorCode::OK;
}

void RecSignal(void*, uint32_t channel)
{
  if (channel < 4u)
  {
    rec.signals[channel]++;
  }
}

void RecReply(void*, uint32_t channel, uint32_t)
{
  if (channel < 4u)
  {
    rec.replies[channel]++;
  }
}

void RecEnable(void*, uint32_t channel, bool)
{
  if (channel < 4u)
  {
    rec.enables[channel]++;
  }
}

void RecCallback(void*)
{
  rec.callbacks++;
}

void RecRegister(void*, uint32_t channel, Mailbox::RxCallback, void*)
{
  if (channel < 4u)
  {
    rec.registers[channel]++;
  }
}

}  // namespace

Mailbox::Channel chans[4];

int main()
{
  /* 1. 五个动词各走各的指针。Each verb reaches its own pointer. */
  Mailbox mbox(RecWait, RecSignal, RecReply, RecEnable, RecRegister, chans, 4u, nullptr);

  const Mailbox::Channel& ch1 = mbox.GetChannel(1u);
  const Mailbox::Channel& ch2 = mbox.GetChannel(2u);
  const Mailbox::Channel& ch3 = mbox.GetChannel(3u);

  ch1.Signal();
  ch2.Reply(0u);
  ch3.SetEnabled(true);
  ch1.RegisterCallback(RecCallback, nullptr);

  TEST_ASSERT(rec.signals[1] == 1);
  TEST_ASSERT(rec.signals[0] == 0 && rec.signals[2] == 0 && rec.signals[3] == 0);
  TEST_ASSERT(rec.replies[2] == 1);
  TEST_ASSERT(rec.replies[1] == 0);
  TEST_ASSERT(rec.enables[3] == 1);
  TEST_ASSERT(rec.enables[1] == 0);
  TEST_ASSERT(rec.registers[1] == 1);
  TEST_ASSERT(rec.registers[0] == 0 && rec.registers[2] == 0);

  /* 2. Wait 报告是哪条通道响的。Wait reports which channel rang. */
  rec.last_channel = 2u;
  uint32_t channel = 0xFFFFFFFFu;
  const ErrorCode result = mbox.Wait(1234u, &channel);
  TEST_ASSERT(result == ErrorCode::OK);
  TEST_ASSERT(channel == 2u);
  TEST_ASSERT(rec.last_timeout == 1234u);

  /* 3. 句柄记住自己是谁。A handle knows which channel it is. */
  TEST_ASSERT(ch1.Id() == 1u);
  TEST_ASSERT(ch2.Id() == 2u);
  TEST_ASSERT(mbox.ChannelCount() == 4u);

  /* 4. 没有的能力是空操作。Absent capabilities are no-ops. */
  const Mailbox bare(RecWait, RecSignal, RecReply, nullptr, nullptr, chans, 4u, nullptr);
  const Mailbox::Channel& bare_ch = bare.GetChannel(1u);
  bare_ch.SetEnabled(false);
  bare_ch.RegisterCallback(RecCallback, nullptr);
  TEST_ASSERT(rec.enables[1] == 0);
  TEST_ASSERT(rec.callbacks == 0);
  TEST_ASSERT(rec.registers[1] == 1); /* the capable one still registered */

  /* 5. channel 为 nullptr 时不写越界内存。A null channel pointer is not written. */
  TEST_ASSERT(mbox.Wait(0u, nullptr) == ErrorCode::OK);
  TEST_ASSERT(rec.last_timeout == 0u);

  return 0;
}
