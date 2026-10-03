#include "linux_shared_page.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <utility>

#include "libxr.hpp"
#include "shared_page.hpp"

namespace LibXR
{
LinuxSharedPage::Mapping LinuxSharedPage::Mapping::Open(uint64_t physical_address)
{
  Mapping mapping;
  if (physical_address == 0)
  {
    return mapping;
  }

  const int64_t system_page_size = ::sysconf(_SC_PAGESIZE);
  if (system_page_size <= 0)
  {
    return mapping;
  }

  const uint64_t page_size = static_cast<uint64_t>(system_page_size);
  const uint64_t page_base = physical_address & ~(page_size - 1U);
  const size_t page_offset = static_cast<size_t>(physical_address - page_base);
  const size_t map_size = ((page_offset + PAGE_SIZE + page_size - 1U) / page_size) *
                          static_cast<size_t>(page_size);

  mapping.fd = ::open("/dev/mem", O_RDWR | O_SYNC);
  if (mapping.fd < 0)
  {
    return mapping;
  }

  mapping.base = ::mmap(nullptr, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mapping.fd,
                        static_cast<off_t>(page_base));
  if (mapping.base == MAP_FAILED)
  {
    mapping.base = nullptr;
    mapping.Reset();
    return mapping;
  }

  mapping.data = static_cast<uint8_t*>(mapping.base) + page_offset;
  mapping.length = map_size;
  return mapping;
}

LinuxSharedPage::Mapping::Mapping(Mapping&& other) noexcept
    : base(other.base), data(other.data), length(other.length), fd(other.fd)
{
  other.base = nullptr;
  other.data = nullptr;
  other.length = 0;
  other.fd = -1;
}

LinuxSharedPage::Mapping& LinuxSharedPage::Mapping::operator=(Mapping&& other) noexcept
{
  if (this != &other)
  {
    Reset();
    base = other.base;
    data = other.data;
    length = other.length;
    fd = other.fd;
    other.base = nullptr;
    other.data = nullptr;
    other.length = 0;
    other.fd = -1;
  }
  return *this;
}

LinuxSharedPage::Mapping::~Mapping() { Reset(); }

void LinuxSharedPage::Mapping::Reset()
{
  if (base != nullptr)
  {
    ::munmap(base, length);
  }
  if (fd >= 0)
  {
    ::close(fd);
  }
  base = nullptr;
  data = nullptr;
  length = 0;
  fd = -1;
}

LinuxSharedPage::LinuxSharedPage(Mapping mapping, Topic topic, uint32_t drain_period_us)
    : SharedPage(mapping.Data(), topic),
      mapping_(std::move(mapping)),
      period_us_(drain_period_us)
{
  ASSERT(drain_period_us != 0);
}

LinuxSharedPage::LinuxSharedPage(uint64_t physical_address, Topic topic,
                                 uint32_t drain_period_us)
    : LinuxSharedPage(Mapping::Open(physical_address), topic, drain_period_us)
{
}

LinuxSharedPage::LinuxSharedPage(uint64_t physical_address, const char* topic_name,
                                 uint32_t drain_period_us)
    : LinuxSharedPage(physical_address, Topic::CreateTopic<TelemetryBatch>(topic_name),
                      drain_period_us)
{
}

LinuxSharedPage::LinuxSharedPage(const SharedPage& page, Topic topic,
                                 uint32_t drain_period_us)
    : SharedPage(page.Data(), topic), period_us_(drain_period_us)
{
  ASSERT(drain_period_us != 0);
}

LinuxSharedPage::LinuxSharedPage(const SharedPage& page, const char* topic_name,
                                 uint32_t drain_period_us)
    : LinuxSharedPage(page, Topic::CreateTopic<TelemetryBatch>(topic_name),
                      drain_period_us)
{
}

bool LinuxSharedPage::Poll(uint64_t now_us)
{
  if (now_us == UINT64_MAX)
  {
    now_us = static_cast<uint64_t>(Timebase::GetMicroseconds());
  }

  if (!clock_started_)
  {
    clock_started_ = true;
  }
  else if (now_us >= last_drain_us_ && now_us - last_drain_us_ < period_us_)
  {
    return false;
  }
  last_drain_us_ = now_us;

  TelemetryBatch batch = {};
  if (Drain(&batch) == 0 && batch.gap == 0)
  {
    return false;
  }

  Publish(batch, MicrosecondTimestamp(now_us));
  return true;
}

uint32_t LinuxSharedPage::Drain(TelemetryBatch* batch)
{
  if (batch == nullptr)
  {
    return 0;
  }
  *batch = {};

  if (!Ready())
  {
    return 0;
  }

  auto reader = TelemetryReader();

  SinceResult result = {};
  const ErrorCode status =
      reader.Since(last_seen_, batch->ring, TELEMETRY_SLOTS, &result);
  if (status == ErrorCode::BUSY)
  {
    return 0;
  }

  last_seen_ = result.next;

  batch->count = result.written;
  batch->head = result.next;
  batch->gap = result.dropped > 0 ? 1U : 0U;
  return result.written;
}

uint32_t LinuxSharedPage::LastSeen() const { return last_seen_; }
}  // namespace LibXR
