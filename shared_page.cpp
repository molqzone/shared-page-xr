#include "shared_page.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include "libxr.hpp"

namespace LibXR
{
namespace
{
constexpr uint32_t READ_RETRIES = 8;

bool try_claim(std::atomic<uint32_t>& state)
{
  uint32_t expected = 0;
  return state.compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                       std::memory_order_relaxed);
}

// 逐字原子拷贝：跨核页面上的字段没有普通读写可言，每 4B 都是一次 atomic_ref 访问。
// 字节数必须是 4 的倍数（MakeLayout 保证）；页面一侧用 uint32_t 视图访问，普通缓冲
// 一侧用 memcpy，不违反混叠规则。
// Word-by-word atomic copy: nothing on the cross-core page has plain accesses; every
// 4B is one atomic_ref operation. The byte counts must be multiples of four, which
// MakeLayout guarantees; the page side is accessed through a uint32_t view and the
// ordinary buffer side through memcpy, so no aliasing rule is bent.
void atomic_copy(void* destination, const void* source, size_t bytes)
{
  for (size_t offset = 0; offset < bytes; offset += sizeof(uint32_t))
  {
    uint32_t word = 0;
    std::memcpy(&word, static_cast<const uint8_t*>(source) + offset, sizeof(word));
    std::atomic_ref<uint32_t>(*reinterpret_cast<uint32_t*>(
                                  static_cast<uint8_t*>(destination) + offset))
        .store(word, std::memory_order_relaxed);
  }
}

void atomic_load_bytes(void* destination, const void* source, size_t bytes)
{
  for (size_t offset = 0; offset < bytes; offset += sizeof(uint32_t))
  {
    const uint32_t word =
        std::atomic_ref<const uint32_t>(
            *reinterpret_cast<const uint32_t*>(static_cast<const uint8_t*>(source) +
                                               offset))
            .load(std::memory_order_relaxed);
    std::memcpy(static_cast<uint8_t*>(destination) + offset, &word, sizeof(word));
  }
}
}  // namespace

namespace detail
{
TelemetryCore::TelemetryCore(void* addr, const PageLayout& layout)
{
  if (addr == nullptr)
  {
    return;
  }

  head_ = reinterpret_cast<std::atomic<uint32_t>*>(
      static_cast<uint8_t*>(addr) + telemetry_offset() +
      static_cast<size_t>(layout.sample_size) * layout.slot_count);
  state_ = reinterpret_cast<std::atomic<uint32_t>*>(
      reinterpret_cast<uint8_t*>(head_) + sizeof(uint32_t));
  ring_ = static_cast<uint8_t*>(addr) + telemetry_offset();
  stride_ = layout.sample_size;
  slots_ = layout.slot_count;
}

uint32_t TelemetryCore::Write(const void* sample)
{
  if (sample == nullptr || head_ == nullptr || state_ == nullptr || ring_ == nullptr)
  {
    return 0;
  }

  uint32_t expected = 0;
  if (!state_->compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                       std::memory_order_relaxed))
  {
    return 0;
  }

  const uint32_t head = head_->load(std::memory_order_relaxed);
  atomic_copy(ring_ + static_cast<size_t>(head % slots_) * stride_, sample, stride_);
  head_->store(head + 1, std::memory_order_release);
  state_->store(0, std::memory_order_release);
  return head + 1;
}

uint32_t TelemetryCore::Head() const
{
  return head_ == nullptr ? 0 : head_->load(std::memory_order_acquire);
}

bool TelemetryCore::Latest(void* out) const
{
  if (out == nullptr || head_ == nullptr || state_ == nullptr || ring_ == nullptr)
  {
    return false;
  }

  for (uint32_t attempt = 0; attempt < READ_RETRIES; ++attempt)
  {
    if (state_->load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    const uint32_t head = head_->load(std::memory_order_acquire);
    if (head == 0)
    {
      return false;
    }

    const uint8_t* slot = ring_ + static_cast<size_t>((head - 1) % slots_) * stride_;
    atomic_load_bytes(out, slot, stride_);
    const uint32_t after = head_->load(std::memory_order_acquire);
    if (after == head && state_->load(std::memory_order_acquire) == 0)
    {
      return true;
    }
  }

  return false;
}

ErrorCode TelemetryCore::Since(uint32_t last_seen, void* out, uint32_t capacity,
                               SinceResult* result) const
{
  if (result == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  *result = {};
  if (head_ == nullptr || state_ == nullptr || ring_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  for (uint32_t attempt = 0; attempt < READ_RETRIES; ++attempt)
  {
    if (state_->load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    const uint32_t head = head_->load(std::memory_order_acquire);
    result->next = head;
    if (head == last_seen)
    {
      return ErrorCode::EMPTY;
    }

    const uint32_t count = head - last_seen;
    if (count > slots_)
    {
      const uint32_t after = head_->load(std::memory_order_acquire);
      if (after != head || state_->load(std::memory_order_acquire) != 0)
      {
        continue;
      }
      result->next = head;
      result->dropped = count;
      return ErrorCode::EMPTY;
    }

    const uint32_t written = out == nullptr ? 0 : ((count < capacity) ? count : capacity);
    if (out != nullptr)
    {
      auto* samples = static_cast<uint8_t*>(out);
      for (uint32_t index = 0; index < written; ++index)
      {
        atomic_load_bytes(samples + static_cast<size_t>(index) * stride_,
                          ring_ +
                              static_cast<size_t>((head - 1 - index) % slots_) * stride_,
                          stride_);
      }
      for (uint32_t first = 0, last = written == 0 ? 0 : written - 1; first < last;
           ++first, --last)
      {
        for (uint32_t offset = 0; offset < stride_; ++offset)
        {
          std::swap(samples[static_cast<size_t>(first) * stride_ + offset],
                    samples[static_cast<size_t>(last) * stride_ + offset]);
        }
      }
    }

    if (head_->load(std::memory_order_acquire) != head ||
        state_->load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    result->written = written;
    result->dropped = count - written;
    return ErrorCode::OK;
  }

  result->next = head_->load(std::memory_order_acquire);
  result->dropped = 0;
  return ErrorCode::BUSY;
}

ReferenceCore::ReferenceCore(void* addr, const PageLayout& layout)
{
  if (addr == nullptr)
  {
    return;
  }

  payload_ = static_cast<uint8_t*>(addr) + layout.region_offset;
  state_ = reinterpret_cast<std::atomic<uint32_t>*>(payload_ + layout.payload_size);
  seq_ = reinterpret_cast<std::atomic<uint32_t>*>(payload_ + layout.payload_size +
                                                 sizeof(uint32_t));
  payload_size_ = layout.payload_size;
}

ErrorCode ReferenceCore::Read(void* out, uint32_t* seq, uint32_t retries) const
{
  if (out == nullptr || payload_ == nullptr || state_ == nullptr || seq_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  for (uint32_t attempt = 0; attempt < retries; ++attempt)
  {
    if (state_->load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    const uint32_t before = seq_->load(std::memory_order_acquire);
    atomic_load_bytes(out, payload_, payload_size_);
    const uint32_t after = seq_->load(std::memory_order_acquire);
    if (before == after && state_->load(std::memory_order_acquire) == 0)
    {
      if (seq != nullptr)
      {
        *seq = after;
      }
      return ErrorCode::OK;
    }
  }

  return ErrorCode::BUSY;
}

uint32_t ReferenceCore::Write(const void* payload)
{
  if (payload_ == nullptr || state_ == nullptr || seq_ == nullptr || payload == nullptr)
  {
    return 0;
  }

  if (!try_claim(*state_))
  {
    return 0;
  }

  const uint32_t next = seq_->load(std::memory_order_relaxed) + 1;
  atomic_copy(payload_, payload, payload_size_);
  seq_->store(next, std::memory_order_release);
  state_->store(0, std::memory_order_release);
  return next;
}

uint32_t ReferenceCore::Seq() const
{
  return seq_ == nullptr ? 0 : seq_->load(std::memory_order_acquire);
}

const void* ReferenceCore::Raw() const { return payload_; }

PageCore::PageCore(void* addr, const PageLayout& layout)
    : page_(static_cast<uint8_t*>(addr)), layout_(layout)
{
}

bool PageCore::Valid() const { return page_ != nullptr; }

uint8_t* PageCore::Data() const { return page_; }

PageMagicKind PageCore::Check() const
{
  if (page_ == nullptr)
  {
    return PageMagicKind::UNFORMATTED;
  }

  const auto* header = reinterpret_cast<const PageHeader*>(page_);
  if (header->magic == PAGE_MAGIC)
  {
    // 逐项互验，与 sg2002_ipc 的 layout_line 检查同构：任何数字对不上都说明双端
    // 帧或 wire 结构漂移，绝不静默错读。
    // Field-by-field cross-check, mirroring the sg2002_ipc layout_line validation: any
    // number that does not match means frame or wire-structure drift between the two
    // ends, and must never be silently misread.
    const bool matches =
        header->abi_version == layout_.abi_version &&
        header->page_size == layout_.page_size &&
        header->slot_count == layout_.slot_count &&
        header->sample_size == layout_.sample_size &&
        header->payload_size == layout_.payload_size &&
        header->region_offset == layout_.region_offset && header->tag == layout_.tag;
    return matches ? PageMagicKind::FORMATTED : PageMagicKind::MISMATCH;
  }

  // A zero page is unformatted; any other invalid header is foreign.
  for (size_t i = 0; i < layout_.page_size; ++i)
  {
    if (page_[i] != 0)
    {
      return PageMagicKind::FOREIGN;
    }
  }
  return PageMagicKind::UNFORMATTED;
}

void PageCore::Format()
{
  if (page_ == nullptr)
  {
    return;
  }

  std::memset(page_, 0, layout_.page_size);
  auto* header = reinterpret_cast<PageHeader*>(page_);
  header->magic = PAGE_MAGIC;
  header->abi_version = layout_.abi_version;
  header->page_size = layout_.page_size;
  header->slot_count = layout_.slot_count;
  header->sample_size = layout_.sample_size;
  header->payload_size = layout_.payload_size;
  header->region_offset = layout_.region_offset;
  header->tag = layout_.tag;
}

void PageCore::ClearHistory()
{
  if (page_ == nullptr)
  {
    return;
  }

  auto* head = reinterpret_cast<std::atomic<uint32_t>*>(
      page_ + telemetry_offset() +
      static_cast<size_t>(layout_.sample_size) * layout_.slot_count);
  auto* telemetry_state =
      reinterpret_cast<std::atomic<uint32_t>*>(reinterpret_cast<uint8_t*>(head) +
                                               sizeof(uint32_t));
  auto* region_state = reinterpret_cast<std::atomic<uint32_t>*>(
      page_ + layout_.region_offset + layout_.payload_size);
  auto* region_seq = reinterpret_cast<std::atomic<uint32_t>*>(
      page_ + layout_.region_offset + layout_.payload_size + sizeof(uint32_t));

  if (!try_claim(*telemetry_state))
  {
    return;
  }
  if (!try_claim(*region_state))
  {
    telemetry_state->store(0, std::memory_order_release);
    return;
  }
  head->store(0, std::memory_order_release);
  region_seq->store(0, std::memory_order_release);
  region_state->store(0, std::memory_order_release);
  telemetry_state->store(0, std::memory_order_release);
}

void PageCore::RecoverStaleClaims()
{
  if (page_ == nullptr)
  {
    return;
  }

  auto* telemetry_state = reinterpret_cast<std::atomic<uint32_t>*>(
      page_ + telemetry_offset() +
      static_cast<size_t>(layout_.sample_size) * layout_.slot_count + sizeof(uint32_t));
  auto* region_state = reinterpret_cast<std::atomic<uint32_t>*>(
      page_ + layout_.region_offset + layout_.payload_size);
  telemetry_state->store(0, std::memory_order_release);
  region_state->store(0, std::memory_order_release);
}
}  // namespace detail
}  // namespace LibXR
