#pragma once

/**
 * @file linux_mapping.hpp
 * @brief Linux 侧的页映射绑定：UIO 设备节点，而不是 `/dev/mem`。
 *        The Linux-side page mapping: a UIO device node, not `/dev/mem`.
 *
 * **平台绑定，不是中立核心**：只有 Linux 消费者 include 本文件它才进入编译；
 * `shared_page.hpp` / `command_channel.hpp` / `doorbell.hpp` 都不引用它。
 * A platform binding, not part of the neutral core: only a Linux consumer includes it.
 *
 * 两页（console、共享页）在设备树里声明成一个 `generic-uio` 节点、两段 `reg`，
 * `uio_pdrv_genirq` 把它们暴露成 `/dev/uioN` 的两个 mmap 索引：索引 0 = console 页，
 * 索引 1 = 共享页（`uio` 用 `vma->vm_pgoff` 当索引，所以偏移是 `index * 页大小`）。
 * 节点**没有中断**：mailbox 的中断 101 被 `cv181x_rtos_cmdqu` 独占，通知走它的
 * `RTOS_CMDQU_WAIT_EVENT`。
 *
 * The two pages (console, shared) are one `generic-uio` node with two `reg` entries; the
 * driver exposes them as two mmap indices on `/dev/uioN`: index 0 is the console page,
 * index 1 the shared page (`uio` treats `vma->vm_pgoff` as the index, so the offset is
 * `index * page size`).  The node carries **no interrupt**: mailbox IRQ 101 is owned
 * exclusively by `cv181x_rtos_cmdqu`, and notifications ride its
 * `RTOS_CMDQU_WAIT_EVENT` ioctl.
 *
 * 这个类也回答**它的一致性域是谁**：`uio` 的 `mmap` 在 `UIO_MEM_PHYS` 上强制
 * `pgprot_noncached`（`drivers/uio/uio.c`），字节直通内存，所以 `Coherence()` 是
 * `no_coherence()`，与 `/dev/mem` 的语义一致——只是不再需要 root 和宽权限的
 * `/dev/mem`。
 *
 * This class also answers *which coherency domain it is*: `uio`'s `mmap` forces
 * `pgprot_noncached` for `UIO_MEM_PHYS` (`drivers/uio/uio.c`), so the bytes go straight
 * to memory and `Coherence()` is `no_coherence()` -- the same semantics as `/dev/mem`,
 * without needing root and a wide-open `/dev/mem`.
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include "shared_page.hpp"

namespace LibXR
{
/** @brief 一个 uio 映射：按 sysfs 里的 uio 名字找到设备，再按 map 索引映射。
 *         One uio mapping: find the device by its sysfs name, then map one index.
 *
 * 全局生命周期约定与库内其它视图一致：映射在整个进程生命周期内有效，不 munmap、
 * 不关 fd；对象是可拷贝的只读视图，`Data() == nullptr` 表示映射失败。
 * Same global-lifetime convention as the rest of the library: the mapping lives for the
 * process lifetime (no munmap, no close), and the object is a copyable view whose
 * `Data() == nullptr` marks a failed mapping.
 */
class UioMapping
{
 public:
  /// @brief 按 uio 名字（`/sys/class/uio/uioN/name`）与 map 索引映射 `bytes` 字节。
  ///        Map `bytes` from the uio device named `name`, at `map_index`.
  static UioMapping Open(const char* name, uint32_t map_index, size_t bytes)
  {
    UioMapping mapping;
    if (name == nullptr || bytes == 0)
    {
      return mapping;
    }

    const int64_t page = ::sysconf(_SC_PAGESIZE);
    if (page <= 0)
    {
      return mapping;
    }

    const std::string device = FindDevice(name);
    if (device.empty())
    {
      return mapping;
    }

    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0)
    {
      return mapping;
    }

    // uio 用 vm_pgoff 当 map 索引，所以第 N 段要从 N 页的偏移开始映射。
    // uio takes vm_pgoff as the map index, so map N starts N pages in.
    const off_t offset = static_cast<off_t>(map_index) * static_cast<off_t>(page);
    void* base = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    if (base == MAP_FAILED)
    {
      // 全局生命周期约定：失败路径不关闭 fd，映射本就不存在。
      // Global-lifetime convention: no close on the failure path; nothing was mapped.
      return mapping;
    }

    mapping.base_ = base;
    mapping.data_ = static_cast<uint8_t*>(base);
    mapping.size_ = bytes;
    mapping.fd_ = fd;
    if (device.size() < sizeof(mapping.path_))
    {
      std::memcpy(mapping.path_, device.c_str(), device.size() + 1);
    }
    return mapping;
  }

  UioMapping() = default;

  [[nodiscard]] uint8_t* Data() const { return data_; }
  [[nodiscard]] size_t Size() const { return size_; }
  /// @brief 映射的 uio 设备节点（诊断用，例如 `/dev/uio0`）。
  ///        The mapped uio node (diagnostics, e.g. `/dev/uio0`).
  [[nodiscard]] const char* DevicePath() const { return path_; }
  explicit operator bool() const { return data_ != nullptr; }

  /// @brief 这个映射的一致性域：non-cacheable，没有对端需要维护。
  ///        The coherency domain: non-cacheable, no peer to maintain.
  ///
  /// 返回的是**策略类型**（`NoCachePolicy`）：页的一致性域在编译期就定了，映射只是
  /// 给出它是哪一个。`Page page(mapping.Data(), mapping.Coherence())` 因此零间接。
  /// Returns the **policy type** (`NoCachePolicy`): a page's domain is fixed at compile
  /// time and the mapping only names it, so `Page page(mapping.Data(), mapping.Coherence())`
  /// costs no indirect call.
  static NoCachePolicy Coherence() { return {}; }

  /// @brief 底层映射基址（交给别的 mmap 包装持有，不额外 munmap）。
  ///        The mapping base, for another mmap view to hold (no extra munmap).
  [[nodiscard]] void* Base() const { return base_; }

 private:
  /// @brief 在 `/sys/class/uio/*/name` 里找同名设备，返回 `/dev/uioN`。
  ///        Find the same-named device under `/sys/class/uio/*/name`, return `/dev/uioN`.
  static std::string FindDevice(const char* name)
  {
    DIR* directory = ::opendir("/sys/class/uio");
    if (directory == nullptr)
    {
      return {};
    }

    std::string found;
    while (dirent* entry = ::readdir(directory))
    {
      // 只认 uioN，跳过 `.`/`..` 和别的条目。
      // Only uioN: skip `.`, `..` and anything else.
      if (std::strncmp(entry->d_name, "uio", 3) != 0 || entry->d_name[3] == '\0')
      {
        continue;
      }

      const std::string path = std::string("/sys/class/uio/") + entry->d_name + "/name";
      std::FILE* file = std::fopen(path.c_str(), "r");
      if (file == nullptr)
      {
        continue;
      }
      char device_name[64] = {};
      const size_t read = std::fread(device_name, 1, sizeof(device_name) - 1, file);
      std::fclose(file);
      while (read != 0 && (device_name[read - 1] == '\n' || device_name[read - 1] == '\r'))
      {
        device_name[read - 1] = '\0';
      }
      if (read == 0 || std::strcmp(device_name, name) != 0)
      {
        continue;
      }

      found = std::string("/dev/") + entry->d_name;
      break;
    }

    ::closedir(directory);
    return found;
  }

  void* base_ = nullptr;
  uint8_t* data_ = nullptr;
  size_t size_ = 0;
  int fd_ = -1;
  char path_[64] = {};
};

}  // namespace LibXR
