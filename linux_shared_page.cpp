#include "linux_shared_page.hpp"

#include "timebase.hpp"

/**
 * @file linux_shared_page.cpp
 * @brief `linux_shared_page.hpp` 的实现。Implementation of `linux_shared_page.hpp`.
 */

namespace LibXR
{
LinuxSharedPage::LinuxSharedPage(SharedPage page, LibXR::Topic topic,
                                 const LinuxSharedPageConfig& config)
    : page_(page), topic_(topic), period_us_(config.drain_period_us)
{
  // 不在这里读时钟：`last_drain_us_ = 0` 让第一次 `Poll()` 确立节律基准，也避免与调用者
  // 注入的时钟（测试）互相干扰。此时页上已有的待发布区间会在第一次 `Poll()` 发出，这正是
  // 启动时该有的行为。
  // No clock is read here: `last_drain_us_ = 0` makes the first `Poll()` establish the
  // cadence baseline and keeps a caller-injected clock (tests) independent. A range
  // already waiting on the page is published by that first `Poll()`, which is what
  // startup should do.
}

bool LinuxSharedPage::Ready() const { return page_.Ready(); }

Reference LinuxSharedPage::Region() { return page_.Region(); }

bool LinuxSharedPage::Poll(uint64_t now_us)
{
  if (now_us == UINT64_MAX)
  {
    now_us = static_cast<uint64_t>(Timebase::GetMicroseconds());
  }

  if (now_us - last_drain_us_ < period_us_)
  {
    return false;
  }
  last_drain_us_ = now_us;

  TelemetryBatch batch = {};
  if (Drain(&batch) == 0 && batch.gap == 0)
  {
    return false;
  }

  topic_.Publish(batch, MicrosecondTimestamp(now_us));
  return true;
}

uint32_t LinuxSharedPage::Drain(TelemetryBatch* batch)
{
  ASSERT(batch != nullptr);
  *batch = {};

  if (!page_.Ready())
  {
    return 0;
  }

  auto reader = page_.TelemetryReader();

  RingScan scan = RingScan::DATA;
  uint32_t next = 0;
  const uint32_t count =
      reader.Since(last_seen_, &scan, &next, batch->ring, TELEMETRY_SLOTS);
  last_seen_ = next;

  if (scan == RingScan::GAP)
  {
    // 整段被覆写：不发采样，只标记缺失区间。
    // The whole range was overwritten: emit no samples, only the missing interval.
    batch->gap = 1;
    batch->head = next;
    return 0;
  }

  batch->count = count;
  batch->head = next;
  return count;
}

uint32_t LinuxSharedPage::WriteAim(bool found, float aim_x, float aim_y)
{
  return page_.Region().WriteAim(found, aim_x, aim_y);
}

uint32_t LinuxSharedPage::WriteParam(uint16_t param_id, float value, uint8_t cmd)
{
  RegionPayload payload = {};
  payload.cmd = cmd;
  payload.param_id = param_id;
  payload.value = value;
  return page_.Region().Write(payload);
}

uint32_t LinuxSharedPage::LastSeen() const { return last_seen_; }
}  // namespace LibXR
