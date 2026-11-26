#pragma once

/**
 * @file console_page.hpp
 * @brief 自描述的 console 页 / The self-describing console page.
 *
 * 页的第一个缓存行同时放环的写指针和一个小描述符：指针仍在偏移 0，描述符从偏移 4
 * 开始，用独立的 magic/version 说明“这一页是什么、环在哪里”。读写两端都 include 本
 * 文件：固件写描述符，Linux 工具（`cop`、`bootlogd`）读描述符，不需要旁边的 JSON。
 *
 * The console page's first cache line holds both the ring's write index and a small
 * descriptor: the index stays at offset 0, the descriptor starts at offset 4 with its
 * own magic/version saying what this page is and where the ring lives.  Both ends
 * include this file: the firmware writes the descriptor and Linux tools (`cop`,
 * `bootlogd`) read it, with no JSON beside it.
 *
 * 描述符写在索引之后的**原有填充**里，所以页布局没有移动：索引 0..3、描述符 4..23、
 * 填充 24..63、环 64..4095。
 * The descriptor occupies the padding that already followed the index, so no offset
 * moves: index 0..3, descriptor 4..23, padding 24..63, ring 64..4095.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "shared_page.hpp"

namespace LibXR
{

/// @brief console 页魔数，内存里是 ASCII `CLOG`。Console page magic, ASCII `CLOG`.
inline constexpr uint32_t CONSOLE_MAGIC = 0x474F4C43U;  // NOLINT

/// @brief 描述符版本。Descriptor version.
inline constexpr uint16_t CONSOLE_DESCRIPTOR_VERSION = 1U;

/// @brief 描述符在页内的偏移（索引 4B 之后）。Offset of the descriptor after the index.
inline constexpr size_t CONSOLE_DESCRIPTOR_OFFSET = 4;

/** @brief console 页描述符，固定在偏移 4。The console descriptor, fixed at offset 4. */
struct ConsoleDescriptor
{
  uint32_t magic;         ///< CONSOLE_MAGIC；最后写。Written last.
  uint16_t version;       ///< CONSOLE_DESCRIPTOR_VERSION。
  uint16_t header_bytes;  ///< sizeof(ConsoleDescriptor)，读者据此跳过本描述符。
  uint32_t index_offset;  ///< 写指针的页内偏移。Offset of the write index.
  uint32_t ring_offset;   ///< 字节环的页内偏移。Offset of the byte ring.
  uint32_t ring_bytes;    ///< 环容量。Ring capacity.
  uint32_t page_bytes;    ///< 这一页的大小，读者据此 mmap。Size of this page.
};

static_assert(sizeof(ConsoleDescriptor) == 24, "console descriptor is 24 bytes");
static_assert(offsetof(ConsoleDescriptor, magic) == 0, "descriptor magic first");
static_assert(offsetof(ConsoleDescriptor, version) == 4, "descriptor version offset");
static_assert(offsetof(ConsoleDescriptor, header_bytes) == 6,
              "descriptor header length offset");
static_assert(offsetof(ConsoleDescriptor, index_offset) == 8, "descriptor index offset");
static_assert(offsetof(ConsoleDescriptor, ring_offset) == 12, "descriptor ring offset");
static_assert(offsetof(ConsoleDescriptor, ring_bytes) == 16, "descriptor ring bytes");
static_assert(offsetof(ConsoleDescriptor, page_bytes) == 20, "descriptor page size offset");

/** @brief console 页：索引、描述符、填充、环。The console page: index, descriptor, pad, ring.
 *
 * `BYTES` 是页大小；`bytes` 从第 2 条缓存行开始，所以索引行与字节行不共享缓存行。
 * `BYTES` is the page size; `bytes` starts on the second cache line, so the index line
 * and the byte line never false-share.
 */
template <size_t BYTES>
struct ConsolePageT
{
  volatile uint32_t write_index;  ///< 偏移 0：累计写入字节数。Offset 0: bytes written.
  ConsoleDescriptor descriptor;   ///< 偏移 4：自描述。Offset 4: the descriptor.
  uint8_t reserved[64u - sizeof(uint32_t) - sizeof(ConsoleDescriptor)];
  alignas(64) volatile uint8_t bytes[BYTES - 64u];
};

static_assert(sizeof(ConsolePageT<4096>) == 4096,
              "the console page is exactly one 4 KiB page");
static_assert(offsetof(ConsolePageT<4096>, write_index) == 0,
              "the index stays at offset 0");
static_assert(offsetof(ConsolePageT<4096>, descriptor) == CONSOLE_DESCRIPTOR_OFFSET,
              "the descriptor follows the index");
static_assert(offsetof(ConsolePageT<4096>, bytes) == 64,
              "the ring starts on line two");

/// @brief 4 KiB console 页的布局常量 / Layout constants of the 4 KiB console page.
inline constexpr size_t CONSOLE_INDEX_OFFSET =
    offsetof(ConsolePageT<4096>, write_index);
inline constexpr size_t CONSOLE_RING_OFFSET = offsetof(ConsolePageT<4096>, bytes);
inline constexpr size_t CONSOLE_RING_BYTES = 4096u - CONSOLE_RING_OFFSET;

/**
 * @brief 运行时的 console 页视图 / Runtime view of the console page.
 *
 * 只读：校验描述符的 magic/version/边界，然后按描述符公布的偏移读索引和环。描述符
 * 无效时 `Valid()` 为 false，`Index()`/`Read()` 返回 0/false，绝不越界。
 * Read-only: validates the descriptor's magic/version/bounds, then reads the index and
 * ring at the advertised offsets.  An invalid descriptor makes `Valid()` false and
 * `Index()`/`Read()` return 0/false; nothing is read out of bounds.
 */
template <Coherent Provider = NoCoherence>
class ConsoleViewT
{
 public:
  ConsoleViewT() = default;

  ConsoleViewT(void* addr, size_t mapped_bytes, const Provider& provider = {})
      : addr_(static_cast<uint8_t*>(addr)), mapped_(mapped_bytes), provider_(provider)
  {
    if (addr_ == nullptr ||
        mapped_ < CONSOLE_DESCRIPTOR_OFFSET + sizeof(ConsoleDescriptor))
    {
      return;
    }
    {
      AccessWindow<Provider> window(provider_, addr_ + CONSOLE_DESCRIPTOR_OFFSET,
                                    sizeof(descriptor_), AccessIntent::READ);
      std::memcpy(&descriptor_, addr_ + CONSOLE_DESCRIPTOR_OFFSET, sizeof(descriptor_));
    }
    valid_ = ValidDescriptor();
  }

  [[nodiscard]] bool Valid() const { return valid_; }
  explicit operator bool() const { return valid_; }

  [[nodiscard]] ConsoleDescriptor Descriptor() const { return descriptor_; }

  [[nodiscard]] uint32_t Index() const
  {
    if (!valid_)
    {
      return 0u;
    }
    uint32_t value = 0u;
    const uint8_t* slot = addr_ + descriptor_.index_offset;
    AccessWindow<Provider> window(provider_, slot, sizeof(value), AccessIntent::READ);
    std::memcpy(&value, slot, sizeof(value));
    return value;
  }

  [[nodiscard]] uint32_t RingOffset() const { return descriptor_.ring_offset; }
  [[nodiscard]] uint32_t RingBytes() const { return descriptor_.ring_bytes; }

  /** @brief 从环内相对偏移 `offset` 读 `bytes`，自动跨环回绕。
   *         Read `bytes` at ring-relative `offset`, wrapping around the ring. */
  bool Read(uint32_t offset, void* out, size_t bytes) const
  {
    if (!valid_ || out == nullptr || descriptor_.ring_bytes == 0u ||
        offset >= descriptor_.ring_bytes)
    {
      return false;
    }
    auto* to = static_cast<uint8_t*>(out);
    size_t remaining = bytes;
    uint32_t at = offset;
    while (remaining != 0u)
    {
      const size_t chunk = remaining < (descriptor_.ring_bytes - at)
                               ? remaining
                               : (descriptor_.ring_bytes - at);
      const uint8_t* src = addr_ + descriptor_.ring_offset + at;
      AccessWindow<Provider> window(provider_, src, chunk, AccessIntent::READ);
      std::memcpy(to, src, chunk);
      to += chunk;
      remaining -= chunk;
      at = 0u;
    }
    return true;
  }

 private:
  [[nodiscard]] bool ValidDescriptor() const
  {
    if (descriptor_.magic != CONSOLE_MAGIC ||
        descriptor_.version != CONSOLE_DESCRIPTOR_VERSION)
    {
      return false;
    }
    if (descriptor_.header_bytes < sizeof(ConsoleDescriptor) ||
        descriptor_.page_bytes == 0u || descriptor_.page_bytes > mapped_ ||
        descriptor_.ring_bytes == 0u)
    {
      return false;
    }
    if (descriptor_.index_offset + sizeof(uint32_t) > descriptor_.page_bytes ||
        descriptor_.ring_offset + descriptor_.ring_bytes > descriptor_.page_bytes)
    {
      return false;
    }
    return true;
  }

  uint8_t* addr_ = nullptr;
  size_t mapped_ = 0;
  ConsoleDescriptor descriptor_ = {};
  bool valid_ = false;
  [[no_unique_address]] Provider provider_{};
};

using ConsoleView = ConsoleViewT<NoCoherence>;

}  // namespace LibXR
