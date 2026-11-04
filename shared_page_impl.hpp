// 本文件是**私有实现头**：模板化的页引擎定义。`shared_page.cpp` 为内置策略显式
// 实例化，消费方要用自己的策略类型时 include 它（例如固件的 C906 策略）。
// A private implementation header: the templated page engine.  `shared_page.cpp`
// explicitly instantiates the built-in policies; a consumer that supplies its own
// policy type includes this file (the firmware's C906 policy does).
#pragma once

#include "shared_page.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>


namespace LibXR
{
namespace detail
{
inline constexpr uint32_t READ_RETRIES = 8;

// 逐字拷贝：跨核页面上的字段没有普通读写可言，每 4B 都是一次 atomic_ref 操作。两侧都
// 走 atomic_ref：页面一侧是必须的原子访存，普通缓冲一侧由 atomic_ref 退化为普通访存。
// 字节数必须是 4 的倍数（make_page_layout 保证）。
// Word-by-word copy: nothing on the cross-core page has plain accesses; every 4B
// is one atomic_ref operation. Both sides go through atomic_ref -- required on
// the page side, degenerating to a plain access on ordinary buffers. The byte
// counts must be multiples of four, which make_page_layout guarantees.
inline void AtomicCopyWords(void* destination, const void* source, size_t bytes)
{
  for (size_t offset = 0; offset < bytes; offset += sizeof(uint32_t))
  {
    const uint32_t word = std::atomic_ref<const uint32_t>(
                              *reinterpret_cast<const uint32_t*>(
                                  static_cast<const uint8_t*>(source) + offset))
                              .load(std::memory_order_relaxed);
    std::atomic_ref<uint32_t>(*reinterpret_cast<uint32_t*>(
                                  static_cast<uint8_t*>(destination) + offset))
        .store(word, std::memory_order_relaxed);
  }
}
}  // namespace detail

template <CoherencePolicyType Policy>
TelemetryCore<Policy>::TelemetryCore(void* addr, const PageLayout& layout,
                             const Policy& coherence)
    : coherence_(coherence)
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

template <CoherencePolicyType Policy>
uint32_t TelemetryCore<Policy>::Write(const void* sample)
{
  if (sample == nullptr || head_ == nullptr || state_ == nullptr || ring_ == nullptr)
  {
    return 0;
  }

  // head_ 与 state_ 相邻（同一条缓存行）：一个 EXCHANGE 窗口覆盖两者的“读改写”，
  // 进入时丢弃过期行，离开时把新的 head 与清零的 claim 一起推回。
  // head_ and state_ are adjacent (one cache line): one EXCHANGE window covers the
  // read-modify-write of both -- stale lines dropped on entry, the new head and the
  // cleared claim pushed back together on exit.
  AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t), AccessIntent::EXCHANGE);

  uint32_t expected = 0;
  if (!state_->compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                       std::memory_order_relaxed))
  {
    return 0;
  }

  const uint32_t head = head_->load(std::memory_order_relaxed);
  auto* slot = ring_ + static_cast<size_t>(head % slots_) * stride_;
  {
    // 样本必须先于 head 对读者可见：槽位窗口先关闭（clean），再写 head。
    // The sample must reach the reader before head advances: the slot window
    // closes (clean) before head is written.
    AccessWindow<Policy> sample_window(coherence_, slot, stride_, AccessIntent::WRITE);
    detail::AtomicCopyWords(slot, sample, stride_);
  }
  head_->store(head + 1, std::memory_order_release);
  state_->store(0, std::memory_order_release);
  return head + 1;
}

template <CoherencePolicyType Policy>
uint32_t TelemetryCore<Policy>::Head() const
{
  if (head_ == nullptr)
  {
    return 0;
  }
  AccessWindow<Policy> read(coherence_, head_, sizeof(uint32_t), AccessIntent::READ);
  return head_->load(std::memory_order_acquire);
}

template <CoherencePolicyType Policy>
bool TelemetryCore<Policy>::Latest(void* out) const
{
  if (out == nullptr || head_ == nullptr || state_ == nullptr || ring_ == nullptr)
  {
    return false;
  }

  for (uint32_t attempt = 0; attempt < detail::READ_RETRIES; ++attempt)
  {
    uint32_t head = 0;
    {
      AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t),
                            AccessIntent::READ);
      if (state_->load(std::memory_order_acquire) != 0)
      {
        continue;
      }
      head = head_->load(std::memory_order_acquire);
      if (head == 0)
      {
        return false;
      }
    }

    const uint8_t* slot = ring_ + static_cast<size_t>((head - 1) % slots_) * stride_;
    {
      AccessWindow<Policy> sample_window(coherence_, slot, stride_, AccessIntent::READ);
      detail::AtomicCopyWords(out, slot, stride_);
    }

    // 复核 head 之前必须重开一个读窗口（再丢一次过期行）：否则可能读到自己缓存里的
    // 旧值，把一次并发发布误判成“没有变化”。
    // Reopen a read window before re-reading head (drop stale lines again): a stale
    // line of our own would otherwise make a concurrent publish look like "nothing
    // changed".
    AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t), AccessIntent::READ);
    const uint32_t after = head_->load(std::memory_order_acquire);
    if (after == head && state_->load(std::memory_order_acquire) == 0)
    {
      return true;
    }
  }

  return false;
}

template <CoherencePolicyType Policy>
ErrorCode TelemetryCore<Policy>::Since(uint32_t last_seen, void* out, uint32_t capacity,
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

  for (uint32_t attempt = 0; attempt < detail::READ_RETRIES; ++attempt)
  {
    uint32_t head = 0;
    {
      AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t),
                            AccessIntent::READ);
      if (state_->load(std::memory_order_acquire) != 0)
      {
        continue;
      }
      head = head_->load(std::memory_order_acquire);
    }
    result->next = head;
    if (head == last_seen)
    {
      return ErrorCode::EMPTY;
    }

    const uint32_t count = head - last_seen;
    if (count > slots_)
    {
      AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t),
                            AccessIntent::READ);
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
        const uint8_t* slot =
            ring_ + static_cast<size_t>((head - 1 - index) % slots_) * stride_;
        AccessWindow<Policy> sample_window(coherence_, slot, stride_, AccessIntent::READ);
        detail::AtomicCopyWords(samples + static_cast<size_t>(index) * stride_, slot, stride_);
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

    {
      AccessWindow<Policy> counters(coherence_, head_, 2 * sizeof(uint32_t),
                            AccessIntent::READ);
      if (head_->load(std::memory_order_acquire) != head ||
          state_->load(std::memory_order_acquire) != 0)
      {
        continue;
      }
    }

    result->written = written;
    result->dropped = count - written;
    return ErrorCode::OK;
  }

  result->next = head_->load(std::memory_order_acquire);
  result->dropped = 0;
  return ErrorCode::BUSY;
}

template <CoherencePolicyType Policy>
ReferenceCore<Policy>::ReferenceCore(void* addr, const PageLayout& layout,
                             const Policy& coherence)
    : coherence_(coherence)
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

template <CoherencePolicyType Policy>
ErrorCode ReferenceCore<Policy>::Read(void* out, uint32_t* seq, uint32_t retries) const
{
  if (out == nullptr || payload_ == nullptr || state_ == nullptr || seq_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  for (uint32_t attempt = 0; attempt < retries; ++attempt)
  {
    // state_ 与 seq_ 相邻（同一条缓存行）：一个读窗口覆盖两者的读。
    // state_ and seq_ are adjacent (one cache line): one read window covers both.
    uint32_t before = 0;
    {
      AccessWindow<Policy> counters(coherence_, state_, 2 * sizeof(uint32_t),
                            AccessIntent::READ);
      if (state_->load(std::memory_order_acquire) != 0)
      {
        continue;
      }
      before = seq_->load(std::memory_order_acquire);
    }

    {
      AccessWindow<Policy> payload_window(coherence_, payload_, payload_size_,
                                  AccessIntent::READ);
      detail::AtomicCopyWords(out, payload_, payload_size_);
    }

    // 复核 seq 之前重开读窗口，理由同遥测读路径。
    // Reopen a read window before re-reading seq, for the same reason as the
    // telemetry read path.
    AccessWindow<Policy> counters(coherence_, state_, 2 * sizeof(uint32_t), AccessIntent::READ);
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

template <CoherencePolicyType Policy>
uint32_t ReferenceCore<Policy>::Write(const void* payload)
{
  if (payload_ == nullptr || state_ == nullptr || seq_ == nullptr || payload == nullptr)
  {
    return 0;
  }

  // claim 是对“可能有过期副本”的读改写：EXCHANGE 进入时 clean+invalidate，
  // 离开时把新的 seq 与释放后的 claim 一起推回。
  // The claim is a read-modify-write over a possibly stale copy: EXCHANGE
  // clean+invalidates on entry and pushes the new seq plus the released claim back
  // on exit.
  AccessWindow<Policy> counters(coherence_, state_, 2 * sizeof(uint32_t),
                        AccessIntent::EXCHANGE);
  uint32_t expected = 0;
  if (!state_->compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                       std::memory_order_relaxed))
  {
    return 0;
  }

  const uint32_t next = seq_->load(std::memory_order_relaxed) + 1;
  {
    // payload 先于 seq 对读者可见：payload 窗口先关闭（clean），再写 seq。
    // The payload must reach the reader before seq advances: the payload window
    // closes (clean) before seq is written.
    AccessWindow<Policy> payload_window(coherence_, payload_, payload_size_,
                                AccessIntent::WRITE);
    detail::AtomicCopyWords(payload_, payload, payload_size_);
  }
  seq_->store(next, std::memory_order_release);
  state_->store(0, std::memory_order_release);
  return next;
}

template <CoherencePolicyType Policy>
uint32_t ReferenceCore<Policy>::Seq() const
{
  if (seq_ == nullptr)
  {
    return 0;
  }
  AccessWindow<Policy> read(coherence_, seq_, sizeof(uint32_t), AccessIntent::READ);
  return seq_->load(std::memory_order_acquire);
}

template <CoherencePolicyType Policy>
const void* ReferenceCore<Policy>::Raw() const { return payload_; }

template <CoherencePolicyType Policy>
PageCore<Policy>::PageCore(void* addr, const PageLayout& layout,
                   const Policy& coherence)
    : page_(static_cast<uint8_t*>(addr)), layout_(layout), coherence_(coherence)
{
}

template <CoherencePolicyType Policy>
bool PageCore<Policy>::Valid() const { return page_ != nullptr; }

template <CoherencePolicyType Policy>
uint8_t* PageCore<Policy>::Data() const { return page_; }

template <CoherencePolicyType Policy>
PageMagicStatus PageCore<Policy>::Check() const
{
  if (page_ == nullptr)
  {
    return PageMagicStatus::UNFORMATTED;
  }

  const auto* header = reinterpret_cast<const PageHeader*>(page_);
  {
    AccessWindow<Policy> header_window(coherence_, page_, sizeof(PageHeader),
                               AccessIntent::READ);
    if (header->magic == PAGE_MAGIC)
    {
      // 逐项互验，与 sg2002_ipc 的 layout_line 检查同构：任何数字对不上都说明双端
      // 帧或 wire 结构漂移，绝不静默错读。
      // Field-by-field cross-check, mirroring the sg2002_ipc layout_line validation:
      // any number that does not match means frame or wire-structure drift between
      // the two ends, and must never be silently misread.
      const bool matches =
          header->abi_version == layout_.abi_version &&
          header->page_size == layout_.page_size &&
          header->slot_count == layout_.slot_count &&
          header->sample_size == layout_.sample_size &&
          header->payload_size == layout_.payload_size &&
          header->region_offset == layout_.region_offset && header->tag == layout_.tag;
      return matches ? PageMagicStatus::FORMATTED : PageMagicStatus::MISMATCH;
    }
  }

  // A zero page is unformatted; any other invalid header is foreign.
  // 这条路径只在不匹配时走一次（诊断用），整页 invalidate 的成本无所谓；但它必须
  // 看到对端写过的每一个字节，否则会把一页非零内容误判成"未格式化"。
  // This path runs once on a mismatch (diagnostics), so a whole-page
  // invalidation costs nothing that matters -- but it must see every byte the
  // peer wrote, or a non-zero page would be misreported as unformatted.
  AccessWindow<Policy> page_window(coherence_, page_, layout_.page_size, AccessIntent::READ);
  for (size_t i = 0; i < layout_.page_size; ++i)
  {
    if (page_[i] != 0)
    {
      return PageMagicStatus::FOREIGN;
    }
  }
  return PageMagicStatus::UNFORMATTED;
}

template <CoherencePolicyType Policy>
void PageCore<Policy>::Format()
{
  if (page_ == nullptr)
  {
    return;
  }

  // 整页重写：一个 WRITE 窗口，离开时把整页脏行推回。
  // The whole page is rewritten: one WRITE window pushes every dirty line back on
  // exit.
  AccessWindow<Policy> page_window(coherence_, page_, layout_.page_size, AccessIntent::WRITE);
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

template <CoherencePolicyType Policy>
void PageCore<Policy>::ClearHistory()
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

  // 两条缓存行覆盖所有被改的计数：head 与遥测 state 相邻，区域 state 与 seq 相邻。
  // Two lines cover every counter touched: head is adjacent to the telemetry state,
  // and the region state to its seq.
  AccessWindow<Policy> telemetry_counters(coherence_, head, 2 * sizeof(uint32_t),
                                  AccessIntent::EXCHANGE);
  AccessWindow<Policy> region_counters(coherence_, region_state, 2 * sizeof(uint32_t),
                               AccessIntent::EXCHANGE);

  uint32_t expected = 0;
  if (!telemetry_state->compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                                std::memory_order_relaxed))
  {
    return;
  }
  expected = 0;
  if (!region_state->compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                             std::memory_order_relaxed))
  {
    telemetry_state->store(0, std::memory_order_release);
    return;
  }
  head->store(0, std::memory_order_release);
  region_seq->store(0, std::memory_order_release);
  region_state->store(0, std::memory_order_release);
  telemetry_state->store(0, std::memory_order_release);
}

template <CoherencePolicyType Policy>
void PageCore<Policy>::RecoverStaleClaims()
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
  // EXCHANGE：先丢掉可能过期的本地行（同一行里还有对端写的 payload/seq），再清
  // claim 并推回——否则会把旧 payload 写回内存，盖掉对端刚写的新值。
  // EXCHANGE drops a possibly stale local line first (the same line also carries
  // the peer's payload/seq), then clears the claim and pushes it back; a plain
  // store+clean would write the stale payload back over the peer's fresh value.
  AccessWindow<Policy> telemetry_claim(coherence_, telemetry_state, sizeof(uint32_t),
                               AccessIntent::EXCHANGE);
  AccessWindow<Policy> region_claim(coherence_, region_state, sizeof(uint32_t),
                            AccessIntent::EXCHANGE);
  telemetry_state->store(0, std::memory_order_release);
  region_state->store(0, std::memory_order_release);
}
}  // namespace LibXR
