/**
 * @file test_console_view.cpp
 * @brief `ConsoleView` 的自描述边界 / Self-describing bounds of `ConsoleView`.
 *
 * 描述符有效时按它公布的偏移读；magic/version/页大小/环越界时 `Valid()` 必须为 false，
 * 并且 `Read()` 不碰越界内存。
 * A valid descriptor is followed; a bad magic/version/page size/ring bound must make
 * `Valid()` false, and `Read()` must not touch out-of-bounds memory.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "console_page.hpp"
#include "test_assert.hpp"

namespace
{
using LibXR::CONSOLE_DESCRIPTOR_VERSION;
using LibXR::CONSOLE_MAGIC;
using LibXR::ConsoleDescriptor;
using LibXR::ConsolePageT;
using LibXR::ConsoleView;

using Storage = std::array<uint8_t, 4096>;

ConsolePageT<4096>* Page(Storage& storage)
{
  return reinterpret_cast<ConsolePageT<4096>*>(storage.data());
}

void PublishDescriptor(Storage& storage)
{
  auto* page = Page(storage);
  page->write_index = 3u;
  page->descriptor.version = CONSOLE_DESCRIPTOR_VERSION;
  page->descriptor.header_bytes = static_cast<uint16_t>(sizeof(ConsoleDescriptor));
  page->descriptor.index_offset = 0u;
  page->descriptor.ring_offset = 64u;
  page->descriptor.ring_bytes = 4096u - 64u;
  page->descriptor.page_bytes = 4096u;
  page->descriptor.magic = CONSOLE_MAGIC;
}

void TestRead()
{
  Storage storage{};
  auto* page = Page(storage);
  page->bytes[0] = static_cast<uint8_t>('a');
  page->bytes[1] = static_cast<uint8_t>('b');
  page->bytes[4095u - 64u] = static_cast<uint8_t>('z');
  PublishDescriptor(storage);

  ConsoleView view(storage.data(), storage.size());
  TEST_ASSERT(view.Valid());
  TEST_ASSERT(view.Index() == 3u);
  TEST_ASSERT(view.RingOffset() == 64u);
  TEST_ASSERT(view.RingBytes() == 4032u);

  char out[3] = {};
  TEST_ASSERT(view.Read(0u, out, 2u));
  TEST_ASSERT(out[0] == 'a' && out[1] == 'b');

  // 从环尾读两个字节：z 之后回到 a。
  // Two bytes starting at the last ring slot: z then back to a.
  TEST_ASSERT(view.Read(4031u, out, 2u));
  TEST_ASSERT(out[0] == 'z' && out[1] == 'a');
}

void TestInvalid()
{
  {
    Storage storage{};
    PublishDescriptor(storage);
    Page(storage)->descriptor.magic = 0u;
    TEST_ASSERT(!ConsoleView(storage.data(), storage.size()).Valid());
  }
  {
    Storage storage{};
    PublishDescriptor(storage);
    Page(storage)->descriptor.version = 2u;
    TEST_ASSERT(!ConsoleView(storage.data(), storage.size()).Valid());
  }
  {
    Storage storage{};
    PublishDescriptor(storage);
    Page(storage)->descriptor.ring_offset = 4090u;
    Page(storage)->descriptor.ring_bytes = 16u;
    TEST_ASSERT(!ConsoleView(storage.data(), storage.size()).Valid());
  }
  {
    Storage storage{};
    PublishDescriptor(storage);
    Page(storage)->descriptor.page_bytes = 8192u;
    TEST_ASSERT(!ConsoleView(storage.data(), storage.size()).Valid());
  }
  {
    Storage storage{};
    PublishDescriptor(storage);
    Page(storage)->descriptor.index_offset = 4094u;
    TEST_ASSERT(!ConsoleView(storage.data(), storage.size()).Valid());
  }
}

}  // namespace

int main()
{
  TestRead();
  TestInvalid();
  return 0;
}
