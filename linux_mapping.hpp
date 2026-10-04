#pragma once

/**
 * @file linux_mapping.hpp
 * @brief Linux reference binding: /dev/mem mapping of one shared page.
 *
 * **平台参考绑定，不是中立核心的一部分**：只有 Linux 消费者 include 本文件，
 * 它才进入编译；中立核心（`shared_page.hpp` / `host_shared_page.hpp`）不引用它。
 * 其他平台按同一形状注入页指针即可（链接脚本符号、静态缓冲、块设备映射……）。
 * Linux reference binding, deliberately not part of the neutral core: only a
 * Linux consumer includes this file, so it enters compilation there; the
 * neutral core does not reference it. Other platforms inject the page pointer
 * in the same shape (linker-script symbol, static buffer, block device
 * mapping, ...).
 *
 * SG2002 场景里 C906L 共享页位于内核保留内存图内，Linux 大核经 `/dev/mem`
 * 映射同一物理页与固件会合；映射地址由 yaml 配置注入，不进任何契约。
 * On SG2002 the C906L shared page lives inside the reserved kernel memory map
 * and the Linux core maps the same physical page through `/dev/mem` to meet
 * the firmware; the physical address is injected through configuration and is
 * not part of any contract.
 */

#include <cstddef>
#include <cstdint>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

namespace LibXR
{

/** @brief RAII `/dev/mem` mapping of one shared page. Linux only. */
class LinuxMapping
{
 public:
  /// @brief Map `page_bytes` at `physical_address`. A failed mapping leaves the
  ///        object invalid; check `Data() != nullptr` before use.
  static LinuxMapping Open(uint64_t physical_address, size_t page_bytes)
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

  LinuxMapping() = default;
  LinuxMapping(const LinuxMapping&) = delete;
  LinuxMapping& operator=(const LinuxMapping&) = delete;

  LinuxMapping(LinuxMapping&& other) noexcept
      : base(other.base), data(other.data), length(other.length), fd(other.fd)
  {
    other.base = nullptr;
    other.data = nullptr;
    other.length = 0;
    other.fd = -1;
  }

  LinuxMapping& operator=(LinuxMapping&& other) noexcept
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

  ~LinuxMapping() { Reset(); }

  [[nodiscard]] uint8_t* Data() const { return data; }

  void Reset()
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

 private:
  void* base = nullptr;
  uint8_t* data = nullptr;
  size_t length = 0;
  int fd = -1;
};

}  // namespace LibXR
