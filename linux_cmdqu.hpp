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

  static constexpr uint8_t kIpSystem = 6u;
  static constexpr uint8_t kDoorbellCmd = 0x7fu;
  static_assert(sizeof(EventWait) == 8u, "WAIT_EVENT argument is 8 bytes");
  static_assert(sizeof(Cmdqu) == 8u, "cmdqu slot is 8 bytes");

  static constexpr unsigned long kWaitEvent = _IOWR('r', 6, EventWait);
  static constexpr unsigned long kSendWait = _IOW('r', 4, unsigned long);
  static constexpr uint16_t kReplyTimeoutMs = 3000u;

  static ErrorCode Wait(void* context, uint32_t timeout_ms, uint32_t* channel)
  {
    auto* self = static_cast<CmdquMailbox*>(context);
    if (self->fd_ < 0)
    {
      return ErrorCode::FAILED;
    }

    EventWait event{self->seq_, timeout_ms};
    if (::ioctl(self->fd_, kWaitEvent, &event) < 0)
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
    (void)context;
    (void)channel;
    // Linux consumes announcements; it does not ring the firmware's event slot.
  }

  static void Reply(void* context, uint32_t channel, uint32_t result)
  {
    auto* self = static_cast<CmdquMailbox*>(context);
    (void)channel;
    if (self->fd_ < 0)
    {
      return;
    }

    Cmdqu reply{kIpSystem, static_cast<uint8_t>(kDoorbellCmd | 0x80u),
                kReplyTimeoutMs, result};
    (void)::ioctl(self->fd_, kSendWait, &reply);
  }

  int fd_ = -1;
  uint32_t seq_ = 0u;
  Mailbox::Channel channels_[1];
  Mailbox mailbox_;
};
}  // namespace LibXR
