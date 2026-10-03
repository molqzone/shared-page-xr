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
namespace detail
{
LinuxMapping LinuxMapping::Open(uint64_t physical_address, size_t page_bytes)
{
  LinuxMapping mapping;
  if (physical_address == 0 || page_bytes == 0)
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
  const size_t map_size = ((page_offset + page_bytes + page_size - 1U) / page_size) *
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

LinuxMapping::LinuxMapping(LinuxMapping&& other) noexcept
    : base(other.base), data(other.data), length(other.length), fd(other.fd)
{
  other.base = nullptr;
  other.data = nullptr;
  other.length = 0;
  other.fd = -1;
}

LinuxMapping& LinuxMapping::operator=(LinuxMapping&& other) noexcept
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

LinuxMapping::~LinuxMapping() { Reset(); }

void LinuxMapping::Reset()
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
}  // namespace detail
}  // namespace LibXR
