/**
 * @file test_command_channel.cpp
 * @brief `CommandChannel` 的命令方向契约 / Command-direction contract of
 *        `CommandChannel`.
 *
 * 通知侧是假的：`Doorbell` 只由三条注入函数指针构成，所以这些测试可以逐条摆出
 * “有通知 / 没通知 / 重复通知”，并检查通道到底回执了什么、有没有越权回执。命令通道
 * 只该用其中两个动词——`Signals()` 那条断言就是钉住“它不会替业务去叫醒对端”。
 * 覆盖：页不可用、通知超时、载荷无效、重复通知重发上次回执、`Ack` 之后才回执、
 * 空参数不消耗通知、通道从不 `Signal()`。
 *
 * The notification side is fake: `Doorbell` is just three injected function pointers, so
 * these tests can stage "notified / not notified / notified again" one case at a time
 * and check exactly what the channel replied -- and that it never replies when nothing
 * is outstanding.  The channel may only use two of the three verbs, which the
 * `Signals()` checks pin down.  Covered: unusable page, notification timeout, invalid
 * payload, a repeated notification replaying the previous reply, reply only through
 * `Ack`, null arguments not consuming a notification, and the channel never signalling.
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "command_channel.hpp"
#include "sample.hpp"
#include "test_assert.hpp"

namespace
{
using LibXRTest::Format;
using LibXRTest::Sample;

/// @brief 测试自有的命令载荷。The test's own command payload.
struct TestPayload
{
  uint32_t magic = 0;
  uint16_t kind = 0;
  uint16_t seq16 = 0;
  uint8_t bytes[16] = {};
};

static_assert(sizeof(TestPayload) == 24);
static_assert(sizeof(TestPayload) % sizeof(uint32_t) == 0);

constexpr uint32_t VALID_MAGIC = 0x31444D47U;

using TestPage = LibXR::SharedPage<Format, Sample, TestPayload, 0, LibXR::NoCoherence>;
using TestChannel =
    LibXR::CommandChannel<Format, Sample, TestPayload, 0, LibXR::NoCoherence>;

/// @brief 通知侧的假实现：脚本化等待结果，记录回执。
///        Fake notification side: scripted wait result, recorded replies.
struct FakePort
{
  LibXR::ErrorCode wait_result = LibXR::ErrorCode::OK;
  unsigned waits = 0;
  unsigned signals = 0;
  unsigned replies = 0;
  uint32_t last_reply = 0;
};

LibXR::ErrorCode FakeWait(void* context, uint32_t)
{
  auto* port = static_cast<FakePort*>(context);
  ++port->waits;
  return port->wait_result;
}

void FakeSignal(void* context)
{
  auto* port = static_cast<FakePort*>(context);
  ++port->signals;
}

void FakeReply(void* context, uint32_t result)
{
  auto* port = static_cast<FakePort*>(context);
  ++port->replies;
  port->last_reply = result;
}

bool ValidPayload(const TestPayload& payload) { return payload.magic == VALID_MAGIC; }

alignas(64) std::array<uint8_t, Format::PAGE_SIZE> g_page_storage{};
alignas(64) std::array<uint8_t, Format::PAGE_SIZE> g_cold_storage{};

TestPayload MakePayload(uint32_t magic, uint16_t kind)
{
  TestPayload payload = {};
  payload.magic = magic;
  payload.kind = kind;
  payload.seq16 = static_cast<uint16_t>(kind);
  return payload;
}

/// 1. 页已格式化、载荷有效：交给业务，回执由 `Ack` 决定（`Wait` 自己不发）。
///    1. Formatted page, valid payload: delivered to the caller; the reply is decided
///    by `Ack`, not by `Wait`.
void test_deliver_then_ack()
{
  TestPage page(g_page_storage.data(), LibXR::NoCoherence{});
  page.Format();
  TEST_ASSERT(page.Region().Write(MakePayload(VALID_MAGIC, 7)) == 1);

  FakePort port;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &seq) == LibXR::ErrorCode::OK);
  TEST_ASSERT(received.magic == VALID_MAGIC);
  TEST_ASSERT(received.kind == 7);
  TEST_ASSERT(seq == 1);
  TEST_ASSERT(port.waits == 1);
  TEST_ASSERT(port.signals == 0);
  TEST_ASSERT(port.replies == 0);

  channel.Ack(seq, 0x55u);
  TEST_ASSERT(port.replies == 1);
  TEST_ASSERT(port.last_reply == 0x55u);
}

/// 2. 载荷无效：回执拒绝，不交给业务，也不记住 seq。
///    2. Invalid payload: rejected with a reply, not delivered, no seq remembered.
void test_invalid_payload_is_rejected()
{
  TestPage page(g_page_storage.data(), LibXR::NoCoherence{});
  page.Format();
  TEST_ASSERT(page.Region().Write(MakePayload(0xDEADBEEFU, 7)) == 1);

  FakePort port;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  received.magic = 0xAAAAAAAAU;
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &seq) == LibXR::ErrorCode::EMPTY);
  TEST_ASSERT(port.replies == 1);
  TEST_ASSERT(port.last_reply == LibXR::COMMAND_REJECTED);
  TEST_ASSERT(received.magic == 0xAAAAAAAAU);
  TEST_ASSERT(seq == 0);
}

/// 3. 页未建立：回执拒绝——发布方负责建页，这里没有页头就没有命令槽。
///    3. Unformatted page: rejected; the publisher owns bringing the page up.
void test_unformatted_page_is_rejected()
{
  TestPage page(g_cold_storage.data(), LibXR::NoCoherence{});

  FakePort port;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &seq) == LibXR::ErrorCode::FAILED);
  TEST_ASSERT(port.replies == 1);
  TEST_ASSERT(port.last_reply == LibXR::COMMAND_REJECTED);
}

/// 4. 重复通知（同一区域 seq）不重放命令，只重发上次回执；新 seq 才再交付。
///    4. A repeated notification (same region seq) does not replay the command; it
///    resends the previous reply. A new seq is delivered again.
void test_repeated_notification_replays_reply()
{
  TestPage page(g_page_storage.data(), LibXR::NoCoherence{});
  page.Format();
  TEST_ASSERT(page.Region().Write(MakePayload(VALID_MAGIC, 9)) == 1);

  FakePort port;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &seq) == LibXR::ErrorCode::OK);
  TEST_ASSERT(seq == 1);
  channel.Ack(seq, 0x99u);
  TEST_ASSERT(port.replies == 1);

  // 没有新写入，doorbell 再来一次：重发 0x99，不交付。
  // No new write, the doorbell rings again: resend 0x99, do not deliver.
  received = {};
  uint32_t duplicate_seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &duplicate_seq) == LibXR::ErrorCode::EMPTY);
  TEST_ASSERT(port.waits == 2);
  TEST_ASSERT(port.signals == 0);
  TEST_ASSERT(port.replies == 2);
  TEST_ASSERT(port.last_reply == 0x99u);
  TEST_ASSERT(duplicate_seq == 0);

  // 新命令（seq 2）照常交付。
  // A new command (seq 2) is delivered as usual.
  TEST_ASSERT(page.Region().Write(MakePayload(VALID_MAGIC, 10)) == 2);
  uint32_t new_seq = 0;
  TEST_ASSERT(channel.Wait(0, &received, &new_seq) == LibXR::ErrorCode::OK);
  TEST_ASSERT(new_seq == 2);
  TEST_ASSERT(received.kind == 10);
}

/// 5. 没有通知就没有在途命令：不回执。
///    5. No notification means nothing outstanding: never reply.
void test_no_notification_never_replies()
{
  TestPage page(g_page_storage.data(), LibXR::NoCoherence{});
  page.Format();
  TEST_ASSERT(page.Region().Write(MakePayload(VALID_MAGIC, 3)) == 1);

  FakePort port;
  port.wait_result = LibXR::ErrorCode::TIMEOUT;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(10, &received, &seq) == LibXR::ErrorCode::TIMEOUT);
  TEST_ASSERT(port.waits == 1);
  TEST_ASSERT(port.signals == 0);
  TEST_ASSERT(port.replies == 0);
  TEST_ASSERT(seq == 0);
}

/// 6. 空参数在消耗通知之前就被拒掉；未绑定的页同样。
///    6. Null arguments are refused before a notification is consumed; so is an
///    unbound page.
void test_null_arguments()
{
  TestPage page(g_page_storage.data(), LibXR::NoCoherence{});
  page.Format();

  FakePort port;
  TestChannel channel(page, LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
                      &ValidPayload);

  TestPayload received = {};
  uint32_t seq = 0;
  TEST_ASSERT(channel.Wait(0, nullptr, &seq) == LibXR::ErrorCode::PTR_NULL);
  TEST_ASSERT(channel.Wait(0, &received, nullptr) == LibXR::ErrorCode::PTR_NULL);
  TEST_ASSERT(port.waits == 0);
  TEST_ASSERT(port.signals == 0);

  TestChannel unbound(
      TestPage(), LibXR::Doorbell{&FakeWait, &FakeSignal, &FakeReply, &port},
      &ValidPayload);
  TEST_ASSERT(unbound.Wait(0, &received, &seq) == LibXR::ErrorCode::PTR_NULL);
  TEST_ASSERT(port.waits == 0);
  TEST_ASSERT(port.signals == 0);
}

}  // namespace

int main()
{
  test_deliver_then_ack();
  test_invalid_payload_is_rejected();
  test_unformatted_page_is_rejected();
  test_repeated_notification_replays_reply();
  test_no_notification_never_replies();
  test_null_arguments();
  return 0;
}
