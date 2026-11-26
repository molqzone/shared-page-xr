#pragma once

#include <cerrno>
#include <cstdint>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "mailbox.hpp"

namespace LibXR
{
/** @brief Linux cmdqu mailbox: one channel, WAIT_EVENT/SEND_WAIT kept private. */
class CmdquMailbox final
{
 public:
  explicit CmdquMailbox(const char* device = "/dev/cvi-rtos-cmdqu")
      : fd_(::open(device, O_RDWR | O_CLOEXEC)),
        mailbox_(&Wait, &Signal, &Reply, nullptr, nullptr, channels_, 1u, this)
  {
  }

  ~CmdquMailbox()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
    }
  }

  CmdquMailbox(const CmdquMailbox&) = delete;
  CmdquMailbox& operator=(const CmdquMailbox&) = delete;

  [[nodiscard]] const Mailbox::Channel& View() const
  {
    return mailbox_.GetChannel(0u);
  }

  /// @brief 设备节点是否打开成功 / Whether the device node opened successfully.
  [[nodiscard]] bool Ready() const { return fd_ >= 0; }

  /// @brief Linux 主动发一条命令并等回执 / Send one command from Linux and wait.
  ///
  /// 这是 `RTOS_CMDQU_SEND_WAIT` 的库包装：`seq` 放进 cmdqu 的 `param`，固件在回执
  /// 里回显它；`*result` 是内核返回的回执字。`Mailbox` 的 `Wait`/`Reply` 只覆盖
  /// 收方向，所以发起方向单独一个动词。
  /// The library wrapper for `RTOS_CMDQU_SEND_WAIT`: `seq` rides in the cmdqu `param`
  /// and the firmware echoes it in the reply; `*result` is the kernel's reply word.
  /// `Mailbox`'s `Wait`/`Reply` cover only the receive direction, so initiation is a
  /// separate verb.
  [[nodiscard]] ErrorCode SendWait(uint32_t seq, uint32_t timeout_ms,
                                   uint32_t* result);

 private:
  struct EventWait
  {
    uint32_t seq;
    uint32_t timeout_ms;
  } __attribute__((packed));

  struct Cmdqu
  {
    uint8_t ip_id;
    uint8_t cmd_block;
    uint16_t block;
    uint32_t param;
  } __attribute__((packed));

  static constexpr uint8_t IP_SYSTEM = 6u;
  static constexpr uint8_t NOTIFY_CMD = 0x7fu;
  static constexpr uint8_t NOTIFY_BLOCK = 1u;
  static_assert(sizeof(EventWait) == 8u, "WAIT_EVENT argument is 8 bytes");
  static_assert(sizeof(Cmdqu) == 8u, "cmdqu slot is 8 bytes");

  static constexpr unsigned long IOCTL_SEND = _IOW('r', 1, unsigned long);
  static constexpr unsigned long IOCTL_WAIT_EVENT = _IOWR('r', 6, EventWait);
  static constexpr unsigned long IOCTL_SEND_WAIT = _IOW('r', 4, unsigned long);
  static constexpr uint16_t REPLY_TIMEOUT_MS = 3000u;

  static ErrorCode Wait(void* context, uint32_t timeout_ms, uint32_t* channel)
  {
    auto* self = static_cast<CmdquMailbox*>(context);
    if (self->fd_ < 0)
    {
      return ErrorCode::FAILED;
    }

    EventWait event{self->seq_, timeout_ms};
    if (::ioctl(self->fd_, IOCTL_WAIT_EVENT, &event) < 0)
    {
      return (errno == EAGAIN || errno == ETIMEDOUT || errno == EINTR)
                 ? ErrorCode::TIMEOUT
                 : ErrorCode::FAILED;
    }

    self->seq_ = event.seq;
    if (channel != nullptr)
    {
      *channel = 0u;
    }
    return ErrorCode::OK;
  }

  static void Signal(void* context, uint32_t channel)
  {
    auto* self = static_cast<CmdquMailbox*>(context);
    (void)channel;
    if (self->fd_ < 0)
    {
      return;
    }

    Cmdqu request{IP_SYSTEM, static_cast<uint8_t>(NOTIFY_CMD | (NOTIFY_BLOCK << 7)),
                  0u, 0u};
    (void)::ioctl(self->fd_, IOCTL_SEND, &request);
  }

  static void Reply(void* context, uint32_t channel, uint32_t result)
  {
    auto* self = static_cast<CmdquMailbox*>(context);
    (void)channel;
    if (self->fd_ < 0)
    {
      return;
    }

    Cmdqu reply{IP_SYSTEM, static_cast<uint8_t>(NOTIFY_CMD | 0x80u),
                REPLY_TIMEOUT_MS, result};
    (void)::ioctl(self->fd_, IOCTL_SEND_WAIT, &reply);
  }

  int fd_ = -1;
  uint32_t seq_ = 0u;
  Mailbox::Channel channels_[1];
  Mailbox mailbox_;
};

inline ErrorCode CmdquMailbox::SendWait(uint32_t seq, uint32_t timeout_ms,
                                        uint32_t* result)
{
  if (fd_ < 0 || result == nullptr)
  {
    return ErrorCode::FAILED;
  }

  Cmdqu command{IP_SYSTEM,
                static_cast<uint8_t>(NOTIFY_CMD | (NOTIFY_BLOCK << 7)),
                static_cast<uint16_t>(timeout_ms > 0xFFFFu ? 0xFFFFu : timeout_ms),
                seq};
  if (::ioctl(fd_, IOCTL_SEND_WAIT, &command) < 0)
  {
    return (errno == EAGAIN || errno == ETIMEDOUT || errno == EINTR)
               ? ErrorCode::TIMEOUT
               : ErrorCode::FAILED;
  }
  *result = command.param;
  return ErrorCode::OK;
}

}  // namespace LibXR
