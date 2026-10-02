#include "shared_page.hpp"

#include <cstring>

#include "crc.hpp"

/**
 * @file shared_page.cpp
 * @brief `shared_page.hpp` 的实现。Implementation of `shared_page.hpp`.
 */

namespace LibXR
{
PageBase::PageBase(void* addr) : page_(static_cast<uint8_t*>(addr)) {}

bool PageBase::Valid() const { return page_ != nullptr; }

uint8_t* PageBase::Data() const { return page_; }

PageMagicKind PageBase::Check() const
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

  // 全零页面视为「刚上电 / 映射错地址」；其余内容视为外来数据。
  // An all-zero page is treated as cold or mis-mapped; anything else is foreign.
  for (size_t i = 0; i < PAGE_SIZE; ++i)
  {
    if (page_[i] != 0)
    {
      return PageMagicKind::FOREIGN;
    }
  }
  return PageMagicKind::UNFORMATTED;
}

void PageBase::Format()
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

void PageBase::ClearHistory()
{
  if (page_ == nullptr)
  {
    return;
  }

  TelemetryRing* telemetry = nullptr;
  Region* region = nullptr;
  Split(&telemetry, &region);
  telemetry->head.store(0, std::memory_order_release);
  region->seq.store(0, std::memory_order_release);
}

void PageBase::Split(TelemetryRing** telemetry, Region** region) const
{
  ASSERT(page_ != nullptr);
  *telemetry = reinterpret_cast<TelemetryRing*>(page_ + TelemetryOffset());
  *region = reinterpret_cast<Region*>(page_ + RegionOffset());
}

Telemetry::Telemetry(void* addr)
    : head_(reinterpret_cast<std::atomic<uint32_t>*>(static_cast<uint8_t*>(addr) +
                                                     TelemetryOffset() +
                                                     offsetof(TelemetryRing, head)))
{
}

uint32_t Telemetry::Write(const Sample& sample)
{
  ASSERT(head_ != nullptr);

  // head_ 指向页内原子对象，物理上就是 mmap 出来的非缓存内存。
  // head_ points at an in-page atomic object backed by non-cached mapped memory.
  const uint32_t head = head_->load(std::memory_order_relaxed);
  const auto* ring = reinterpret_cast<const Sample*>(
      reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
  auto* slot = const_cast<Sample*>(&ring[head % TELEMETRY_SLOTS]);
  *slot = sample;
  head_->store(head + 1, std::memory_order_release);
  return head + 1;
}

uint32_t Telemetry::Head() const
{
  return head_ == nullptr ? 0 : head_->load(std::memory_order_acquire);
}

bool Telemetry::Latest(Sample* sample) const
{
  if (sample == nullptr || head_ == nullptr)
  {
    return false;
  }

  const uint32_t head = head_->load(std::memory_order_acquire);
  if (head == 0)
  {
    return false;
  }

  const auto* ring = reinterpret_cast<const Sample*>(
      reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
  *sample = ring[(head - 1) % TELEMETRY_SLOTS];
  return true;
}

uint32_t Telemetry::Since(uint32_t last_seen, RingScan* scan, uint32_t* next,
                          Sample* samples, uint32_t capacity) const
{
  ASSERT(scan != nullptr);
  ASSERT(next != nullptr);
  ASSERT(head_ != nullptr);

  const uint32_t head = head_->load(std::memory_order_acquire);

  if (head == last_seen)
  {
    *scan = RingScan::IDLE;
    *next = head;
    return 0;
  }

  if (head - last_seen > TELEMETRY_SLOTS)
  {
    *scan = RingScan::GAP;
    *next = head;
    return 0;
  }

  const uint32_t count = head - last_seen;

  // 写者可能在区间判定与拷贝之间继续推进并覆写最旧槽，所以按最新优先倒序拷贝，让区间里
  // 最旧的一条尽早读到，撕裂窗口最小。
  // The writer may advance between the range decision and the copies, overwriting the
  // oldest slots, so the range is copied newest first to read its oldest sample as early
  // as possible.
  const auto* ring = reinterpret_cast<const Sample*>(
      reinterpret_cast<const uint8_t*>(head_) - offsetof(TelemetryRing, head));
  if (samples != nullptr)
  {
    const uint32_t limit = count < capacity ? count : capacity;
    for (uint32_t i = 0; i < limit; ++i)
    {
      samples[i] = ring[(head - 1 - i) % TELEMETRY_SLOTS];
    }
    // 倒序读入后翻正，调用者拿到正序区间。
    // Reverse the newest-first copies so the caller sees the range in order.
    for (uint32_t i = 0, j = (limit == 0) ? 0 : limit - 1; i < j; ++i, --j)
    {
      const Sample tmp = samples[i];
      samples[i] = samples[j];
      samples[j] = tmp;
    }
  }

  *scan = RingScan::DATA;
  *next = head;
  return count;
}

uint32_t Telemetry::SeekToHead() const { return Head(); }

Reference::Reference(void* addr)
    : region_(reinterpret_cast<Region*>(static_cast<uint8_t*>(addr) + RegionOffset()))
{
}

RegionScan Reference::Read(Region* out, uint32_t retries) const
{
  ASSERT(out != nullptr);
  ASSERT(region_ != nullptr);

  for (uint32_t attempt = 0; attempt < retries; ++attempt)
  {
    const uint32_t before = region_->seq.load(std::memory_order_acquire);
    out->payload = region_->payload;
    const uint32_t after = region_->seq.load(std::memory_order_acquire);
    if (before == after)
    {
      out->seq.store(after, std::memory_order_relaxed);
      return RegionScan::CURRENT;
    }
  }

  // 重试上限内未读到自洽快照（写者持续发布）：返回最后一次拷贝并标记 BUSY。
  // No coherent snapshot within the retry budget (the writer keeps publishing): return
  // the last copy tagged BUSY.
  out->payload = region_->payload;
  out->seq.store(region_->seq.load(std::memory_order_acquire), std::memory_order_relaxed);
  return RegionScan::BUSY;
}

uint32_t Reference::Seq() const
{
  return region_ == nullptr ? 0 : region_->seq.load(std::memory_order_acquire);
}

uint32_t Reference::Write(const RegionPayload& payload)
{
  ASSERT(region_ != nullptr);

  region_->payload = payload;
  const uint32_t next = region_->seq.load(std::memory_order_relaxed) + 1;
  region_->seq.store(next, std::memory_order_release);
  return next;
}

uint32_t Reference::WriteAim(bool found, float aim_x, float aim_y)
{
  RegionPayload payload = {};
  payload.found = found ? 1U : 0U;
  payload.aim_x = aim_x;
  payload.aim_y = aim_y;
  return Write(payload);
}

const Region* Reference::Raw() const { return region_; }

SharedPage::SharedPage(void* addr) : PageBase(addr) {}

bool SharedPage::Ready() const { return Check() == PageMagicKind::FORMATTED; }

Telemetry SharedPage::TelemetryWriter() { return Telemetry(Data()); }

Telemetry SharedPage::TelemetryWriter() const
{
  return Telemetry(const_cast<uint8_t*>(Data()));
}

Telemetry SharedPage::TelemetryReader() const { return TelemetryWriter(); }

Reference SharedPage::Region() { return Reference(Data()); }

Reference SharedPage::Region() const { return Reference(const_cast<uint8_t*>(Data())); }

uint32_t SharedPage::WriteSample(const Sample& sample)
{
  return TelemetryWriter().Write(sample);
}

bool SharedPage::Latest(Sample* sample) const { return TelemetryReader().Latest(sample); }

AccessUnitPage::AccessUnitPage(void* addr) : PageBase(addr) {}

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
  slot->crc32.store(0, std::memory_order_release);
  slot->reserved = 0;
}

PageMagicKind AccessUnitPage::Check() const
{
  if (Data() == nullptr)
  {
    return PageMagicKind::UNFORMATTED;
  }
  return Slot()->magic == ACCESS_UNIT_MAGIC ? PageMagicKind::FORMATTED
                                            : PageMagicKind::UNFORMATTED;
}

uint32_t AccessUnitPage::Publish(const void* data, uint32_t length, uint32_t format,
                                 uint32_t width, uint32_t height, bool compute_crc32)
{
  if (Data() == nullptr || data == nullptr || length > AccessUnit::MAX_BYTES)
  {
    return 0;
  }

  auto* slot = Slot();
  std::memcpy(slot->payload, data, length);
  slot->format = format;
  slot->width = width;
  slot->height = height;
  slot->crc32.store(compute_crc32 ? CRC32::Calculate(data, length) : 0,
                    std::memory_order_relaxed);
  slot->length.store(length, std::memory_order_relaxed);
  slot->ready.store(1, std::memory_order_relaxed);

  // release：保证 payload/头部字段对取帧者可见再公开 seq。
  // release: make the payload and header fields visible before publishing seq.
  const uint32_t next = slot->seq.load(std::memory_order_relaxed) + 1;
  slot->seq.store(next, std::memory_order_release);
  return next;
}

AccessUnitPage::View AccessUnitPage::Acquire(bool verify_crc32, uint32_t retries)
{
  View view = {};
  if (Data() == nullptr)
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
    const uint32_t before = slot->seq.load(std::memory_order_acquire);
    const uint32_t length = slot->length.load(std::memory_order_acquire);
    if (length > AccessUnit::MAX_BYTES)
    {
      break;
    }
    const uint32_t after = slot->seq.load(std::memory_order_acquire);
    if (before == after)
    {
      view.data = slot->payload;
      view.length = length;
      view.format = slot->format;
      view.width = slot->width;
      view.height = slot->height;
      view.seq = before;

      // 校验失败不动 ready：该帧留待重试，而不是被当成已消费。
      // A failed check leaves `ready` set, so the frame can be retried instead of
      // counting as consumed.
      const uint32_t stored = slot->crc32.load(std::memory_order_relaxed);
      if (verify_crc32 && stored != 0 && CRC32::Calculate(view.data, length) != stored)
      {
        return View{};
      }

      slot->ready.store(0, std::memory_order_release);
      return view;
    }
  }

  return view;
}

AccessUnit* AccessUnitPage::Slot() const
{
  return reinterpret_cast<AccessUnit*>(const_cast<uint8_t*>(Data()) + Offset());
}

}  // namespace LibXR
