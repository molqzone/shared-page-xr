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

/** @brief `/dev/mem` mapping of one shared page. Linux only.
 *
 * 全局生命周期约定（与 LibXR 的分配一次、永不释放一致）：映射在进程生命周期内
 * 有效，不 munmap、不关 fd，析构时什么也不做——所以这个类干脆没有析构函数。
 * 对象本身是可拷贝的只读视图（与库内其它视图同风格），拷贝即同一映射的另一个
 * 视图；`Data() == nullptr` 表示映射失败。
 * Global-lifetime convention (matching the LibXR allocate-once, never-release
 * rule): the mapping lives for the process lifetime -- no munmap, no close,
 * nothing at destruction time, so this class simply has no destructor. The
 * object is a copyable read-only view (same style as the library's other
 * views); copying yields another view of the same mapping. `Data() == nullptr`
 * marks a failed mapping.
 */
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
      // Leave data null: the caller's validity check rejects the object. The
      // fd and any partial state stay as they are under the global-lifetime
      // convention; this is a one-time bind failure path.
      mapping.base = nullptr;
      return mapping;
    }

    mapping.data = static_cast<uint8_t*>(mapping.base) + page_offset;
    mapping.length = map_size;
    return mapping;
  }

  LinuxMapping() = default;

  [[nodiscard]] uint8_t* Data() const { return data; }

 private:
  void* base = nullptr;
  uint8_t* data = nullptr;
  size_t length = 0;
  int fd = -1;
};

}  // namespace LibXR
