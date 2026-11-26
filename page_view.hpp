#pragma once

/**
 * @file page_view.hpp
 * @brief 从自描述页头运行时读取一页 / Read a self-describing page at runtime.
 *
 * `SharedPage<Format, T, P, TAG>` 把几何编译进两端；本文件是它的**运行时对应物**：
 * 只拿 `PageHeader` 与映射长度，推导出环/通道表/命令区的偏移，不需要知道 `Format`、
 * `T` 或 `P`。通用的 Linux 工具（`cop`）因此不必链接产品契约。
 *
 * The compile-time counterpart is `SharedPage<Format, T, P, TAG>`; this file is the
 * runtime one: it derives the ring/channel-table/command-region offsets from the
 * `PageHeader` and the mapping length alone, without knowing `Format`, `T` or `P`.  A
 * generic Linux tool (`cop`) therefore needs no product contract.
 *
 * 写入仍走同一套 `write_state`/`seq` 协议：`PageView` 复用 `TelemetryCore` /
 * `ReferenceCore`（`shared_page.cpp` 里的显式实例化），所以没有第二份同步实现。
 * Writes use the same `write_state`/`seq` protocol: `PageView` reuses `TelemetryCore` /
 * `ReferenceCore` (explicitly instantiated in `shared_page.cpp`), so there is no second
 * synchronization implementation.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "shared_page.hpp"

namespace LibXR
{

/** @brief 运行时页布局：`PageLayout` + 由它推导出的区域偏移。
 *         Runtime page layout: `PageLayout` plus the region offsets derived from it.
 */
struct RuntimeLayout : PageLayout
{
  uint32_t telemetry_offset = 0;
  uint32_t head_offset = 0;
  uint32_t write_state_offset = 0;
  uint32_t channels_offset = 0;
  uint32_t channel_capacity = 0;
  uint32_t seq_offset = 0;

  /**
   * @brief 从页头推导并校验运行时布局 / Derive and validate the runtime layout.
   *
   * 所有偏移都必须落在页内、互不重叠，且命令区正好以 `write_state` + `seq` 收尾。
   * 任何一条不成立都返回 false，调用方把它当作 `MISMATCH`，绝不静默错读。
   */
  static bool Parse(const PageHeader& header, size_t mapped_bytes, RuntimeLayout* out);
};

inline bool RuntimeLayout::Parse(const PageHeader& header, size_t mapped_bytes,
                                 RuntimeLayout* out)
{
  if (out == nullptr || header.magic != PAGE_MAGIC)
  {
    return false;
  }
  if (header.header_bytes < sizeof(PageHeader) ||
      header.header_bytes > PAGE_CACHE_LINE)
  {
    return false;
  }
  if (header.page_size < sizeof(PageHeader) || header.page_size > mapped_bytes ||
      header.page_size % sizeof(uint32_t) != 0)
  {
    return false;
  }
  if (header.slot_count < 2 || header.sample_size < sizeof(uint32_t) ||
      header.sample_size % sizeof(uint32_t) != 0)
  {
    return false;
  }
  if (header.payload_size < sizeof(CommandHeader) ||
      header.payload_size % sizeof(uint32_t) != 0)
  {
    return false;
  }
  if (header.region_offset < sizeof(PageHeader) ||
      header.region_offset % sizeof(uint32_t) != 0 ||
      header.region_offset + header.payload_size + 2 * sizeof(uint32_t) !=
          header.page_size)
  {
    return false;
  }

  const uint64_t ring_end = static_cast<uint64_t>(LibXR::telemetry_offset()) +
                            static_cast<uint64_t>(header.slot_count) *
                                header.sample_size;
  if (ring_end + 2 * sizeof(uint32_t) > header.region_offset)
  {
    return false;
  }

  RuntimeLayout layout{};
  layout.page_size = header.page_size;
  layout.slot_count = header.slot_count;
  layout.sample_size = header.sample_size;
  layout.payload_size = header.payload_size;
  layout.region_offset = header.region_offset;
  layout.tag = header.tag;
  layout.command_magic = header.command_magic;
  layout.header_bytes = header.header_bytes;
  layout.telemetry_offset = static_cast<uint32_t>(LibXR::telemetry_offset());
  layout.head_offset = static_cast<uint32_t>(ring_end);
  layout.write_state_offset = layout.head_offset + sizeof(uint32_t);
  layout.seq_offset = header.region_offset + header.payload_size + sizeof(uint32_t);

  const size_t channels =
      align_up(static_cast<size_t>(ring_end) + 2 * sizeof(uint32_t), PAGE_CACHE_LINE);
  layout.channels_offset =
      static_cast<uint32_t>(channels > header.region_offset ? header.region_offset
                                                            : channels);
  if (layout.channels_offset + sizeof(ChannelTableHeader) <= header.region_offset)
  {
    layout.channel_capacity =
        static_cast<uint32_t>((header.region_offset - layout.channels_offset -
                               sizeof(ChannelTableHeader)) /
                              sizeof(ChannelEntry));
  }
  else
  {
    layout.channel_capacity = 0;
  }

  *out = layout;
  return true;
}

/** @brief 容量未知的通道表视图 / A channel-table view with no compile-time capacity. */
template <Coherent Provider>
class ChannelTableView
{
 public:
  ChannelTableView() = default;

  ChannelTableView(void* addr, const RuntimeLayout& layout, const Provider& provider)
      : table_(addr == nullptr ? nullptr
                               : static_cast<uint8_t*>(addr) + layout.channels_offset),
        capacity_(layout.channel_capacity),
        provider_(provider)
  {
  }

  [[nodiscard]] bool Valid() const
  {
    ChannelTableHeader header{};
    return ReadHeader(&header) && header.magic == CHANNEL_TABLE_MAGIC &&
           header.count <= capacity_;
  }
  explicit operator bool() const { return Valid(); }

  [[nodiscard]] uint32_t Count() const
  {
    ChannelTableHeader header{};
    return ReadHeader(&header) && header.magic == CHANNEL_TABLE_MAGIC &&
                   header.count <= capacity_
               ? header.count
               : 0u;
  }

  [[nodiscard]] uint32_t Generation() const
  {
    ChannelTableHeader header{};
    return ReadHeader(&header) && header.magic == CHANNEL_TABLE_MAGIC &&
                   header.count <= capacity_
               ? header.gen
               : 0u;
  }

  bool Entry(uint32_t index, ChannelEntry* out) const
  {
    ChannelTableHeader header{};
    if (out == nullptr || !ReadHeader(&header) ||
        header.magic != CHANNEL_TABLE_MAGIC || header.count > capacity_ ||
        index >= header.count)
    {
      return false;
    }
    const uint8_t* entry = table_ + sizeof(ChannelTableHeader) +
                           static_cast<size_t>(index) * sizeof(ChannelEntry);
    AccessWindow<Provider> window(provider_, entry, sizeof(ChannelEntry),
                                  AccessIntent::READ);
    std::memcpy(out, entry, sizeof(ChannelEntry));
    return true;
  }

 private:
  bool ReadHeader(ChannelTableHeader* out) const
  {
    if (table_ == nullptr || capacity_ == 0 || out == nullptr)
    {
      return false;
    }
    AccessWindow<Provider> window(provider_, table_, sizeof(*out), AccessIntent::READ);
    std::memcpy(out, table_, sizeof(*out));
    return true;
  }

  uint8_t* table_ = nullptr;
  uint32_t capacity_ = 0;
  [[no_unique_address]] Provider provider_{};
};

/**
 * @brief 运行时的一页：读遥测、写命令、列出通道 / One page at runtime.
 *
 * `PageView` 不拥有映射；它只借用 `addr` 与 `mapped_bytes`，生命周期由调用方保证
 * （`cop` 的 UIO 映射是进程级的，测试用匿名映射）。
 * `PageView` does not own the mapping; it borrows `addr`/`mapped_bytes` for the caller's
 * lifetime (the `cop` UIO mapping is process-lifetime; tests use anonymous pages).
 */
template <Coherent Provider, typename Words = LrScWords>
class PageView
{
 public:
  PageView() = default;

  PageView(void* addr, size_t mapped_bytes, const Provider& provider = {})
      : addr_(static_cast<uint8_t*>(addr)), mapped_(mapped_bytes), provider_(provider)
  {
    if (addr_ == nullptr || mapped_ < sizeof(PageHeader))
    {
      return;
    }
    {
      AccessWindow<Provider> window(provider_, addr_, sizeof(PageHeader),
                                    AccessIntent::READ);
      std::memcpy(&header_, addr_, sizeof(header_));
    }
    if (header_.magic != PAGE_MAGIC ||
        !RuntimeLayout::Parse(header_, mapped_, &runtime_))
    {
      return;
    }

    command_buffer_.assign(runtime_.payload_size, 0u);
    telemetry_ = TelemetryCore<Provider, Words>(addr_, runtime_, provider_);
    reference_ = ReferenceCore<Provider, Words>(addr_, runtime_, provider_);
    valid_ = true;
  }

  [[nodiscard]] bool Valid() const { return valid_; }
  explicit operator bool() const { return valid_; }

  /** @brief 与 `PageCore::Check()` 同义的运行时版本。
   *         The runtime counterpart of `PageCore::Check()`.
   *
   * 魔术字正确但布局不自洽时是 `MISMATCH`（工具与固件/页面版本不一致），全零页是
   * `UNFORMATTED`，非零的陌生页是 `FOREIGN`。
   * A correct magic with an inconsistent layout is `MISMATCH` (tool and firmware/page
   * version disagree), an all-zero page is `UNFORMATTED`, and a non-zero unknown page
   * is `FOREIGN`.
   */
  [[nodiscard]] PageMagicStatus Check() const
  {
    if (addr_ == nullptr || mapped_ < sizeof(PageHeader))
    {
      return PageMagicStatus::UNFORMATTED;
    }

    PageHeader header{};
    {
      AccessWindow<Provider> window(provider_, addr_, sizeof(PageHeader),
                                    AccessIntent::READ);
      std::memcpy(&header, addr_, sizeof(header));
    }
    if (header.magic == PAGE_MAGIC)
    {
      return valid_ ? PageMagicStatus::FORMATTED : PageMagicStatus::MISMATCH;
    }

    AccessWindow<Provider> window(provider_, addr_, mapped_, AccessIntent::READ);
    for (size_t index = 0; index < mapped_; ++index)
    {
      if (addr_[index] != 0u)
      {
        return PageMagicStatus::FOREIGN;
      }
    }
    return PageMagicStatus::UNFORMATTED;
  }

  [[nodiscard]] PageHeader Header() const { return header_; }
  [[nodiscard]] const RuntimeLayout& Layout() const { return runtime_; }

  [[nodiscard]] uint32_t Head() const { return valid_ ? telemetry_.Head() : 0u; }

  bool Latest(void* slot) const { return valid_ && telemetry_.Latest(slot); }

  [[nodiscard]] ErrorCode Since(uint32_t last_seen, void* slots, uint32_t capacity,
                                SinceResult* result) const
  {
    return valid_ ? telemetry_.Since(last_seen, slots, capacity, result)
                  : ErrorCode::FAILED;
  }

  [[nodiscard]] uint32_t Seq() const { return valid_ ? reference_.Seq() : 0u; }

  /**
   * @brief 写一条通用命令：8B 头 + 不透明 payload / Write a generic command.
   *
   * 头里的 magic 取页头的 `command_magic`，seq 取页 seq 的下一个值；payload 是产品
   * 私有字节，工具不解释。`payload_size - 8` 是上限。
   * The header's magic comes from the page header's `command_magic` and its seq is the
   * next page sequence; the payload is product-private bytes the tool does not interpret.
   * `payload_size - 8` is the limit.
   */
  [[nodiscard]] ErrorCode WriteCommand(uint16_t type, const void* payload,
                                       size_t payload_bytes, uint32_t* seq)
  {
    if (!valid_ || seq == nullptr || runtime_.command_magic == 0u ||
        runtime_.payload_size < sizeof(CommandHeader))
    {
      return ErrorCode::FAILED;
    }
    if (payload_bytes > runtime_.payload_size - sizeof(CommandHeader) ||
        (payload_bytes != 0u && payload == nullptr))
    {
      return ErrorCode::FAILED;
    }

    std::fill(command_buffer_.begin(), command_buffer_.end(), 0u);
    const uint32_t predicted = reference_.Seq() + 1u;
    auto* header = reinterpret_cast<CommandHeader*>(command_buffer_.data());
    header->magic = runtime_.command_magic;
    header->type = type;
    header->seq = static_cast<uint16_t>(predicted);
    if (payload_bytes != 0u)
    {
      std::memcpy(command_buffer_.data() + sizeof(CommandHeader), payload, payload_bytes);
    }

    const uint32_t written = reference_.Write(command_buffer_.data());
    if (written == 0u)
    {
      return ErrorCode::BUSY;
    }
    *seq = written;
    return ErrorCode::OK;
  }

  [[nodiscard]] ChannelTableView<Provider> Channels() const
  {
    return ChannelTableView<Provider>(addr_, runtime_, provider_);
  }

 private:
  uint8_t* addr_ = nullptr;
  size_t mapped_ = 0;
  PageHeader header_ = {};
  RuntimeLayout runtime_ = {};
  bool valid_ = false;
  std::vector<uint8_t> command_buffer_{};
  TelemetryCore<Provider, Words> telemetry_{};
  ReferenceCore<Provider, Words> reference_{};
  [[no_unique_address]] Provider provider_{};
};

}  // namespace LibXR
