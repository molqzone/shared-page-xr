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
 * **索引不写死**：一个节点有几段 `reg` 是设备树说的，不是驱动说的，所以
 * `OpenNamed()` 按 sysfs 的 `maps/mapN/name` 找名字（这里是 `shared`），大小取
 * `maps/mapN/size`。写死 `map 1` 只在"这一段就是我想的那一段"时成立，而这句话要么由
 * 设备树保证，要么就得读出来——`bsp-guidance-vision` 早先就是这么做的，它手写了
 * 这份查找；现在搬进来，两份合成一份。
 *
 * **The index is not written down**: how many `reg` entries a node has is the device
 * tree's decision, not the driver's, so `OpenNamed()` finds the map by its sysfs name
 * (`shared` here) and takes its size from `maps/mapN/size`.  Hard-coding `map 1` is only
 * right when that really is the page, which is either a device-tree guarantee or
 * something to read -- `bsp-guidance-vision` used to read it, with its own copy of this
 * lookup; that copy now lives here.
 *
 * 这个类也回答**它的一致性域是谁**：`uio` 的 `mmap` 在 `UIO_MEM_PHYS` 上强制
 * `pgprot_noncached`（`drivers/uio/uio.c`），字节直通内存，所以 `Coherence()` 是
 * `NoCoherence{}`，与 `/dev/mem` 的语义一致——只是不再需要 root 和宽权限的
 * `/dev/mem`。
 *
 * This class also answers *which coherency domain it is*: `uio`'s `mmap` forces
 * `pgprot_noncached` for `UIO_MEM_PHYS` (`drivers/uio/uio.c`), so the bytes go straight
 * to memory and `Coherence()` is `NoCoherence{}` -- the same semantics as `/dev/mem`,
 * without needing root and a wide-open `/dev/mem`.
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
/// @brief uio 的 sysfs 发现：只读属性文件，不做 mmap，因此能在主机上测。
///        uio discovery through sysfs: attribute files only, no mmap, so it is testable
///        on a host.
///
/// 这三个函数都收一个 `root`（真机是 `/sys/class/uio`），测试给一棵假树就行。
/// All three take a `root` (the real one is `/sys/class/uio`), so a test can hand them a
/// fake tree.
namespace uio
{
/// @brief 读一个 sysfs 属性并去掉行尾空白 / Read one sysfs attribute, trimming the end.
inline std::string ReadAttribute(const std::string& path)
{
  std::FILE* file = std::fopen(path.c_str(), "r");
  if (file == nullptr)
  {
    return {};
  }
  char buffer[64] = {};
  const size_t read = std::fread(buffer, 1, sizeof(buffer) - 1, file);
  std::fclose(file);
  size_t end = read;
  while (end != 0 && (buffer[end - 1] == '\n' || buffer[end - 1] == '\r' ||
                      buffer[end - 1] == ' ' || buffer[end - 1] == '\t'))
  {
    --end;
  }
  return std::string(buffer, end);
}

/// @brief 按设备名找 sysfs 目录名（`uioN`），找不到返回空串。
///        Find the sysfs dir name (`uioN`) for a device name; empty when absent.
inline std::string FindDevice(const char* root, const char* name)
{
  if (root == nullptr || name == nullptr)
  {
    return {};
  }
  DIR* directory = ::opendir(root);
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
    if (ReadAttribute(std::string(root) + "/" + entry->d_name + "/name") != name)
    {
      continue;
    }
    found = entry->d_name;
    break;
  }

  ::closedir(directory);
  return found;
}

/// @brief 一段映射：索引与字节数 / One map entry: index and byte count.
struct MapInfo
{
  uint32_t index = 0;
  size_t bytes = 0;
};

/// @brief 读 `maps/mapN/size`；读不到或为 0 时返回 false。
///        Read `maps/mapN/size`; false when it is absent or zero.
inline bool ReadMapSize(const char* root, const char* device, uint32_t index, size_t* bytes)
{
  if (root == nullptr || device == nullptr)
  {
    return false;
  }
  const std::string path = std::string(root) + "/" + device + "/maps/map" +
                           std::to_string(index) + "/size";
  const std::string text = ReadAttribute(path);
  if (text.empty())
  {
    return false;
  }
  const unsigned long long value = std::strtoull(text.c_str(), nullptr, 0);
  if (value == 0)
  {
    return false;
  }
  if (bytes != nullptr)
  {
    *bytes = static_cast<size_t>(value);
  }
  return true;
}

/// @brief 按 `maps/mapN/name` 找一段映射 / Find a map by its `maps/mapN/name`.
inline bool FindMapByName(const char* root, const char* device, const char* map_name,
                          MapInfo* out)
{
  if (root == nullptr || device == nullptr || map_name == nullptr)
  {
    return false;
  }
  // uio 的段数没有上限属性可读，扫到第一个不存在的 mapN 就停。
  // There is no "map count" attribute, so stop at the first mapN that does not exist.
  for (uint32_t index = 0; index < 8; ++index)
  {
    const std::string base =
        std::string(root) + "/" + device + "/maps/map" + std::to_string(index);
    const std::string name = ReadAttribute(base + "/name");
    if (name.empty())
    {
      break;
    }
    if (name != map_name)
    {
      continue;
    }
    size_t bytes = 0;
    (void)ReadMapSize(root, device, index, &bytes);
    if (out != nullptr)
    {
      out->index = index;
      out->bytes = bytes;
    }
    return true;
  }
  return false;
}
}  // namespace uio

/** @brief 一个 uio 映射：按 sysfs 里的 uio 名字找到设备，再映射其中一段。
 *         One uio mapping: find the device by its sysfs name, then map one entry.
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
  ///
  /// 索引写死只适合"这段就是我要的"由设备树保证的场合；一页多段时用 `OpenNamed()`。
  /// A fixed index only suits a node whose layout the device tree guarantees; use
  /// `OpenNamed()` when one node carries several entries.
  static UioMapping Open(const char* name, uint32_t map_index, size_t bytes)
  {
    const std::string device = uio::FindDevice(kSysfsRoot, name);
    if (device.empty() || bytes == 0)
    {
      return {};
    }
    return Bind(device, map_index, bytes, 0);
  }

  /// @brief 按 sysfs 里的 map 名字选映射，字节数取 sysfs 的 `size`。
  ///        Map the entry whose sysfs name is `map_name`, at the size sysfs reports.
  ///
  /// 名字查不到时退回 `fallback_index` + `fallback_bytes`（老内核的 uio 没有
  /// `name`/`size` 属性），所以"设备树改了段序"不会让这里静默读错一页。
  /// `O_SYNC` 打开：这份内存是对端在轮询的，写出去就该已经在内存里。
  /// When the name is not found it falls back to `fallback_index` + `fallback_bytes`
  /// (older kernels expose neither `name` nor `size`), so a device tree that reorders
  /// its entries does not turn into a silent read of the wrong page.  The node is
  /// opened `O_SYNC`: the peer polls this memory, so a write should already be in
  /// memory.
  static UioMapping OpenNamed(const char* name, const char* map_name, uint32_t fallback_index,
                              size_t fallback_bytes)
  {
    const std::string device = uio::FindDevice(kSysfsRoot, name);
    if (device.empty())
    {
      return {};
    }
    uio::MapInfo info;
    if (!uio::FindMapByName(kSysfsRoot, device.c_str(), map_name, &info))
    {
      info.index = fallback_index;
      size_t bytes = 0;
      if (uio::ReadMapSize(kSysfsRoot, device.c_str(), fallback_index, &bytes))
      {
        info.bytes = bytes;
      }
    }
    const size_t bytes = info.bytes != 0 ? info.bytes : fallback_bytes;
    if (bytes == 0)
    {
      return {};
    }
    return Bind(device, info.index, bytes, O_SYNC);
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
  /// 返回的是 **provider 类型**（`NoCoherence`）：页的一致性域在编译期就定了，映射只是
  /// 给出它是哪一个。`Page page(mapping.Data(), mapping.Coherence())` 因此零间接。
  /// Returns the **provider type** (`NoCoherence`): a page's domain is fixed at compile
  /// time and the mapping only names it, so `Page page(mapping.Data(), mapping.Coherence())`
  /// costs no indirect call.
  static NoCoherence Coherence() { return {}; }

  /// @brief 底层映射基址（交给别的 mmap 包装持有，不额外 munmap）。
  ///        The mapping base, for another mmap view to hold (no extra munmap).
  [[nodiscard]] void* Base() const { return base_; }

 private:
  /// @brief 真机上的 sysfs 入口 / The sysfs entry point on a real machine.
  inline static const char* kSysfsRoot = "/sys/class/uio";

  /// @brief 打开 `/dev/<device>` 并映射第 `map_index` 段。
  ///        Open `/dev/<device>` and map entry `map_index`.
  static UioMapping Bind(const std::string& device, uint32_t map_index, size_t bytes,
                         int extra_flags)
  {
    UioMapping mapping;
    const int64_t page = ::sysconf(_SC_PAGESIZE);
    if (page <= 0)
    {
      return mapping;
    }

    const std::string path = std::string("/dev/") + device;
    const int fd = ::open(path.c_str(), O_RDWR | extra_flags);
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
    if (path.size() < sizeof(mapping.path_))
    {
      std::memcpy(mapping.path_, path.c_str(), path.size() + 1);
    }
    return mapping;
  }

  void* base_ = nullptr;
  uint8_t* data_ = nullptr;
  size_t size_ = 0;
  int fd_ = -1;
  char path_[64] = {};
};

}  // namespace LibXR
