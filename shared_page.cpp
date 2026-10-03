#include "shared_page.hpp"

#include <atomic>
#include <cstdint>
#include <cstring>

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

void atomic_word_store(uint32_t* destination, uint32_t value)
{
  std::atomic_ref<uint32_t>(*destination).store(value, std::memory_order_relaxed);
}

uint32_t atomic_word_load(const uint32_t* source)
{
  return std::atomic_ref<const uint32_t>(*source).load(std::memory_order_relaxed);
}

template <size_t Bytes>
void atomic_store(void* destination, const void* source)
{
  static_assert(Bytes % sizeof(uint32_t) == 0);
  uint32_t words[Bytes / sizeof(uint32_t)];
  std::memcpy(words, source, Bytes);
  auto* target = static_cast<uint32_t*>(destination);
  for (size_t index = 0; index < Bytes / sizeof(uint32_t); ++index)
  {
    std::atomic_ref<uint32_t>(target[index])
        .store(words[index], std::memory_order_relaxed);
  }
}

template <size_t Bytes>
void atomic_load(void* destination, const void* source)
{
  static_assert(Bytes % sizeof(uint32_t) == 0);
  uint32_t words[Bytes / sizeof(uint32_t)];
  const auto* source_words = static_cast<const uint32_t*>(source);
  for (size_t index = 0; index < Bytes / sizeof(uint32_t); ++index)
  {
    words[index] = std::atomic_ref<const uint32_t>(source_words[index])
                       .load(std::memory_order_relaxed);
  }
  std::memcpy(destination, words, Bytes);
}
}  // namespace

SharedPage::SharedPage(void* addr) : Topic(), page_(static_cast<uint8_t*>(addr)) {}

SharedPage::SharedPage(void* addr, Topic topic)
    : Topic(topic), page_(static_cast<uint8_t*>(addr))
{
}

bool SharedPage::Valid() const { return page_ != nullptr; }

uint8_t* SharedPage::Data() const { return page_; }

PageMagicKind SharedPage::Check() const
{
  if (page_ == nullptr)
  {
    return PageMagicKind::UNFORMATTED;
  }

  const auto* nodes = reinterpret_cast<const PageHeader*>(page_);
  if (nodes->magic == PAGE_MAGIC && nodes->page_size == PAGE_SIZE)
  {
    return PageMagicKind::FORMATTED;
  }

  // A zero page is unformatted; any other invalid header is foreign.
  for (size_t i = 0; i < PAGE_SIZE; ++i)
  {
    if (page_[i] != 0)
    {
      return PageMagicKind::FOREIGN;
    }
  }
  return PageMagicKind::UNFORMATTED;
}

void SharedPage::Format()
{
  if (page_ == nullptr)
  {
    return;
  }

  std::memset(page_, 0, PAGE_SIZE);
  auto* nodes = reinterpret_cast<PageHeader*>(page_);
  nodes->magic = PAGE_MAGIC;
  nodes->page_size = PAGE_SIZE;
}

void SharedPage::ClearHistory()
{
  if (page_ == nullptr)
  {
    return;
  }

  auto* telemetry = reinterpret_cast<TelemetryRing*>(page_ + telemetry_offset());
  auto* reference = reinterpret_cast<LibXR::Region*>(page_ + region_offset());
  if (!try_claim(telemetry->write_state))
  {
    return;
  }
  if (!try_claim(reference->write_state))
  {
    telemetry->write_state.store(0, std::memory_order_release);
    return;
  }
  telemetry->head.store(0, std::memory_order_release);
  reference->seq.store(0, std::memory_order_release);
  reference->write_state.store(0, std::memory_order_release);
  telemetry->write_state.store(0, std::memory_order_release);
}

Telemetry::Telemetry(void* addr)
{
  if (addr == nullptr)
  {
    return;
  }

  auto* telemetry =
      reinterpret_cast<TelemetryRing*>(static_cast<uint8_t*>(addr) + telemetry_offset());
  head_ = &telemetry->head;
  state_ = &telemetry->write_state;
  ring_ = telemetry->ring;
}

uint32_t Telemetry::Write(const Sample& sample)
{
  if (head_ == nullptr || state_ == nullptr || ring_ == nullptr)
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
  atomic_store<sizeof(Sample)>(&ring_[head % TELEMETRY_SLOTS], &sample);
  head_->store(head + 1, std::memory_order_release);
  state_->store(0, std::memory_order_release);
  return head + 1;
}

uint32_t Telemetry::Head() const
{
  return head_ == nullptr ? 0 : head_->load(std::memory_order_acquire);
}

bool Telemetry::Latest(Sample* sample) const
{
  if (sample == nullptr || head_ == nullptr || state_ == nullptr || ring_ == nullptr)
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

    Sample value = {};
    atomic_load<sizeof(Sample)>(&value, &ring_[(head - 1) % TELEMETRY_SLOTS]);
    const uint32_t after = head_->load(std::memory_order_acquire);
    if (after == head && state_->load(std::memory_order_acquire) == 0)
    {
      *sample = value;
      return true;
    }
  }

  return false;
}

ErrorCode Telemetry::Since(uint32_t last_seen, Sample* samples, uint32_t capacity,
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
    if (count > TELEMETRY_SLOTS)
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

    const uint32_t written =
        samples == nullptr ? 0 : ((count < capacity) ? count : capacity);
    if (samples != nullptr)
    {
      for (uint32_t index = 0; index < written; ++index)
      {
        atomic_load<sizeof(Sample)>(&samples[index],
                                    &ring_[(head - 1 - index) % TELEMETRY_SLOTS]);
      }
      for (uint32_t first = 0, last = written == 0 ? 0 : written - 1; first < last;
           ++first, --last)
      {
        const Sample value = samples[first];
        samples[first] = samples[last];
        samples[last] = value;
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

uint32_t Telemetry::SeekToHead() const { return Head(); }

Reference::Reference(void* addr)
    : region_(addr == nullptr ? nullptr
                              : reinterpret_cast<Region*>(static_cast<uint8_t*>(addr) +
                                                          region_offset()))
{
}

ErrorCode Reference::Read(Region* out, uint32_t retries) const
{
  if (out == nullptr || region_ == nullptr)
  {
    return ErrorCode::PTR_NULL;
  }

  for (uint32_t attempt = 0; attempt < retries; ++attempt)
  {
    if (region_->write_state.load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    const uint32_t before = region_->seq.load(std::memory_order_acquire);
    uint8_t payload[REGION_PAYLOAD_BYTES] = {};
    atomic_load<REGION_PAYLOAD_BYTES>(payload, region_->payload);
    const uint32_t after = region_->seq.load(std::memory_order_acquire);
    if (before == after && region_->write_state.load(std::memory_order_acquire) == 0)
    {
      std::memcpy(out->payload, payload, REGION_PAYLOAD_BYTES);
      out->write_state.store(0, std::memory_order_relaxed);
      out->seq.store(after, std::memory_order_relaxed);
      return ErrorCode::OK;
    }
  }

  return ErrorCode::BUSY;
}

uint32_t Reference::Seq() const
{
  return region_ == nullptr ? 0 : region_->seq.load(std::memory_order_acquire);
}

uint32_t Reference::Write(const void* payload, size_t size)
{
  if (region_ == nullptr || payload == nullptr || size > REGION_PAYLOAD_BYTES)
  {
    return 0;
  }

  uint8_t value[REGION_PAYLOAD_BYTES] = {};
  std::memcpy(value, payload, size);

  if (!try_claim(region_->write_state))
  {
    return 0;
  }

  const uint32_t next = region_->seq.load(std::memory_order_relaxed) + 1;
  atomic_store<REGION_PAYLOAD_BYTES>(region_->payload, value);
  region_->seq.store(next, std::memory_order_release);
  region_->write_state.store(0, std::memory_order_release);
  return next;
}

const Region* Reference::Raw() const { return region_; }

bool SharedPage::Ready() const { return Check() == PageMagicKind::FORMATTED; }

Telemetry SharedPage::TelemetryWriter() { return Telemetry(Data()); }

Telemetry SharedPage::TelemetryWriter() const { return Telemetry(Data()); }

Telemetry SharedPage::TelemetryReader() const { return TelemetryWriter(); }

Reference SharedPage::Region() { return Reference(Data()); }

Reference SharedPage::Region() const { return Reference(Data()); }

uint32_t SharedPage::WriteSample(const Sample& sample)
{
  return TelemetryWriter().Write(sample);
}

bool SharedPage::Latest(Sample* sample) const { return TelemetryReader().Latest(sample); }

AccessUnitPage::AccessUnitPage(void* addr) : page_(static_cast<uint8_t*>(addr)) {}

bool AccessUnitPage::Valid() const { return page_ != nullptr; }

uint8_t* AccessUnitPage::Data() const { return page_; }

void AccessUnitPage::Format()
{
  if (Data() == nullptr)
  {
    return;
  }

  auto* slot = Slot();
  slot->magic = ACCESS_UNIT_MAGIC;
  slot->format = AccessUnit::FORMAT_UNKNOWN;
  slot->width = 0;
  slot->height = 0;
  slot->seq.store(0, std::memory_order_release);
  slot->length.store(0, std::memory_order_release);
  slot->ready.store(0, std::memory_order_release);
  slot->write_state.store(0, std::memory_order_release);
}

PageMagicKind AccessUnitPage::Check() const
{
  if (Data() == nullptr)
  {
    return PageMagicKind::UNFORMATTED;
  }
  if (Slot()->magic == ACCESS_UNIT_MAGIC)
  {
    return PageMagicKind::FORMATTED;
  }
  return Slot()->magic == 0 ? PageMagicKind::UNFORMATTED : PageMagicKind::FOREIGN;
}

uint32_t AccessUnitPage::Publish(const void* data, uint32_t length, uint32_t format,
                                 uint32_t width, uint32_t height)
{
  if (Data() == nullptr || data == nullptr || length > AccessUnit::MAX_BYTES)
  {
    return 0;
  }

  auto* slot = Slot();
  uint32_t expected = 0;
  if (!slot->write_state.compare_exchange_strong(expected, 1, std::memory_order_acquire,
                                                 std::memory_order_relaxed))
  {
    return 0;
  }

  slot->ready.store(0, std::memory_order_release);
  std::memcpy(slot->payload, data, length);
  atomic_word_store(&slot->format, format);
  atomic_word_store(&slot->width, width);
  atomic_word_store(&slot->height, height);
  slot->length.store(length, std::memory_order_relaxed);

  const uint32_t next = slot->seq.load(std::memory_order_relaxed) + 1;
  slot->seq.store(next, std::memory_order_release);
  slot->ready.store(1, std::memory_order_release);
  slot->write_state.store(0, std::memory_order_release);
  return next;
}

AccessUnitPage::View AccessUnitPage::Acquire(uint32_t retries)
{
  View view = {};
  if (Data() == nullptr || Check() != PageMagicKind::FORMATTED)
  {
    return view;
  }

  auto* slot = Slot();
  if (slot->ready.load(std::memory_order_acquire) == 0)
  {
    return view;
  }

  for (uint32_t attempt = 0; attempt < retries; ++attempt)
  {
    if (slot->write_state.load(std::memory_order_acquire) != 0)
    {
      continue;
    }

    const uint32_t before = slot->seq.load(std::memory_order_acquire);
    const uint32_t length = slot->length.load(std::memory_order_acquire);
    if (length > AccessUnit::MAX_BYTES)
    {
      break;
    }
    const uint32_t after = slot->seq.load(std::memory_order_acquire);
    if (before == after && slot->write_state.load(std::memory_order_acquire) == 0)
    {
      view.data = slot->payload;
      view.length = length;
      view.format = atomic_word_load(&slot->format);
      view.width = atomic_word_load(&slot->width);
      view.height = atomic_word_load(&slot->height);
      view.seq = before;

      if (slot->seq.load(std::memory_order_acquire) != before ||
          slot->write_state.load(std::memory_order_acquire) != 0)
      {
        continue;
      }

      uint32_t expected = 1;
      if (slot->ready.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                              std::memory_order_acquire))
      {
        return view;
      }
      return View{};
    }
  }

  return view;
}

AccessUnit* AccessUnitPage::Slot() const
{
  return Data() == nullptr ? nullptr : reinterpret_cast<AccessUnit*>(Data() + Offset());
}

}  // namespace LibXR
