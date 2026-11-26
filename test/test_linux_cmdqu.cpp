/**
 * @file test_linux_cmdqu.cpp
 * @brief `CmdquMailbox::SendWait` 的失败路径 / Failure paths of `SendWait`.
 *
 * 没有真实的 cmdqu 设备时，`Ready()` 必须是 false，`SendWait()` 必须返回 FAILED 而不是
 * 去写一个未映射的槽；空 result 同理。成功路径需要板子，主机上不伪造 ioctl。
 * Without a real cmdqu device, `Ready()` must be false and `SendWait()` must return
 * FAILED instead of writing an unmapped slot; a null result is the same.  The success
 * path needs the board, so the host does not fake the ioctl.
 */

#include <cstdint>

#include "linux_cmdqu.hpp"
#include "test_assert.hpp"

int main()
{
  LibXR::CmdquMailbox mailbox("/nonexistent/cvi-rtos-cmdqu");
  TEST_ASSERT(!mailbox.Ready());

  uint32_t reply = 0;
  TEST_ASSERT(mailbox.SendWait(1u, 10u, &reply) == LibXR::ErrorCode::FAILED);
  TEST_ASSERT(mailbox.SendWait(1u, 10u, nullptr) == LibXR::ErrorCode::FAILED);
  return 0;
}
