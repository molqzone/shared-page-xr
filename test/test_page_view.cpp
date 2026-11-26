/**
 * @file test_page_view.cpp
 * @brief `PageView` 的运行时契约 / Runtime contract of `PageView`.
 *
 * 同一页由 `SharedPage`（编译期几何）建立，再由 `PageView`（只读页头）读回；两边必须
 * 得到同一组偏移、同一条遥测、同一代通道表。`PageView::WriteCommand` 写的字节必须
 * 能被 `SharedPage::Region()` 读成同一条命令。
 *
 * The same page is built by `SharedPage` (compile-time geometry) and read back by
 * `PageView` (header only); both must agree on the layout, the latest sample and the
 * channel table.  What `PageView::WriteCommand` writes must read back through
 * `SharedPage::Region()` as the same command.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "page_view.hpp"
#include "sample.hpp"
#include "shared_page.hpp"
#include "test_assert.hpp"

namespace
{
using LibXR::ChannelAnnouncement;
using LibXR::ChannelEntry;
using LibXR::ErrorCode;
using LibXR::NoCoherence;
using LibXR::PageHeader;
using LibXR::PageMagicStatus;
using LibXR::PageView;
using LibXR::SharedPage;
using LibXRTest::make_sample;
using LibXRTest::same_sample;
using Sample = LibXRTest::Sample;

/// 带命令魔数的测试几何 / Test geometry with a command magic.
struct CommandFormat : LibXRTest::Format
{
  static constexpr uint32_t COMMAND_MAGIC = 0x31444D47U;
};

/// 通用命令 payload：8B 头 + 16B 不透明数据 / Generic command payload.
struct CommandPayload
{
  uint32_t magic = 0;
  uint16_t type = 0;
  uint16_t seq = 0;
  uint8_t bytes[16] = {};
};

static_assert(sizeof(CommandPayload) == 24, "command payload is 24 bytes");
static_assert(sizeof(CommandPayload) % sizeof(uint32_t) == 0,
              "the region copies whole words");

constexpr uint32_t TAG = 0x54414731U;
using TestPage = SharedPage<CommandFormat, Sample, CommandPayload, TAG, NoCoherence>;
using TestView = PageView<NoCoherence>;

using Storage = std::array<uint8_t, CommandFormat::PAGE_SIZE>;

void TestReadBack()
{
  Storage storage{};
  TestPage page(storage.data(), NoCoherence{});
  page.Format();
  TEST_ASSERT(page.Check() == PageMagicStatus::FORMATTED);
  TEST_ASSERT(page.WriteSample(make_sample(1)) == 1);
  TEST_ASSERT(page.WriteSample(make_sample(2)) == 2);

  TestView view(storage.data(), storage.size());
  TEST_ASSERT(view.Valid());
  TEST_ASSERT(view.Check() == PageMagicStatus::FORMATTED);

  // 页头 + 推导出的偏移必须和编译期布局一致。
  // The header plus derived offsets must equal the compile-time layout.
  const auto& layout = view.Layout();
  TEST_ASSERT(layout.page_size == CommandFormat::PAGE_SIZE);
  TEST_ASSERT(layout.slot_count == CommandFormat::SLOT_COUNT);
  TEST_ASSERT(layout.sample_size == sizeof(Sample));
  TEST_ASSERT(layout.payload_size == sizeof(CommandPayload));
  TEST_ASSERT(layout.region_offset ==
              (LibXR::region_offset<CommandFormat, CommandPayload>()));
  TEST_ASSERT(layout.telemetry_offset == LibXR::telemetry_offset());
  TEST_ASSERT(layout.head_offset ==
               LibXR::telemetry_offset() + CommandFormat::SLOT_COUNT * sizeof(Sample));
  TEST_ASSERT(layout.write_state_offset == layout.head_offset + 4u);
  TEST_ASSERT(layout.seq_offset == layout.region_offset + sizeof(CommandPayload) + 4u);

  TEST_ASSERT(view.Head() == 2u);
  Sample latest{};
  TEST_ASSERT(view.Latest(&latest));
  TEST_ASSERT(same_sample(latest, make_sample(2)));
}

void TestChannels()
{
  Storage storage{};
  TestPage page(storage.data(), NoCoherence{});
  page.Format();
  const ChannelAnnouncement announcements[] = {
      {"imu", 0x400u, LibXR::CHANNEL_FLAG_PRESENT},
      {"gyro", 0x401u, 0u},
  };
  TEST_ASSERT(page.Channels<4>().Publish(announcements, 2));

  TestView view(storage.data(), storage.size());
  auto channels = view.Channels();
  TEST_ASSERT(channels.Valid());
  TEST_ASSERT(channels.Count() == 2u);
  TEST_ASSERT(channels.Generation() == 1u);

  ChannelEntry entry{};
  TEST_ASSERT(channels.Entry(0u, &entry));
  TEST_ASSERT(entry.name_len == 3u);
  TEST_ASSERT(std::memcmp(entry.name, "imu", 3u) == 0);
  TEST_ASSERT(entry.endpoint == 0x400u);
  TEST_ASSERT(channels.Entry(1u, &entry));
  TEST_ASSERT(entry.name_len == 4u);
  TEST_ASSERT(std::memcmp(entry.name, "gyro", 4u) == 0);
  TEST_ASSERT(!channels.Entry(2u, &entry));
}

void TestWriteCommand()
{
  Storage storage{};
  TestPage page(storage.data(), NoCoherence{});
  page.Format();

  TestView view(storage.data(), storage.size());
  uint8_t payload[16] = {};
  for (uint8_t index = 0; index < sizeof(payload); ++index)
  {
    payload[index] = static_cast<uint8_t>(0xA0u + index);
  }

  uint32_t seq = 0;
  TEST_ASSERT(view.WriteCommand(7u, payload, sizeof(payload), &seq) == ErrorCode::OK);
  TEST_ASSERT(seq == 1u);

  CommandPayload command{};
  uint32_t observed = 0;
  TEST_ASSERT(page.Region().Read(&command, &observed) == ErrorCode::OK);
  TEST_ASSERT(observed == seq);
  TEST_ASSERT(command.magic == CommandFormat::COMMAND_MAGIC);
  TEST_ASSERT(command.type == 7u);
  TEST_ASSERT(command.seq == static_cast<uint16_t>(seq));
  TEST_ASSERT(std::memcmp(command.bytes, payload, sizeof(payload)) == 0);

  // 超长 payload 被拒绝，不写半条命令。
  // An oversized payload is refused; no half command is written.
  uint8_t too_large[17] = {};
  TEST_ASSERT(view.WriteCommand(7u, too_large, sizeof(too_large), &seq) ==
               ErrorCode::FAILED);
}

void TestStates()
{
  {
    Storage storage{};
    TestView view(storage.data(), storage.size());
    TEST_ASSERT(view.Check() == PageMagicStatus::UNFORMATTED);
  }
  {
    Storage storage{};
    storage[0] = 0xEFu;
    storage[1] = 0xBEu;
    storage[2] = 0xADu;
    storage[3] = 0xDEu;
    TestView view(storage.data(), storage.size());
    TEST_ASSERT(view.Check() == PageMagicStatus::FOREIGN);
  }
  {
    Storage storage{};
    TestPage page(storage.data(), NoCoherence{});
    page.Format();
    // 让 region 与 page 尾对不上：magic 仍对，但布局不自洽 -> MISMATCH。
    // Break the region/page-tail equality while keeping the magic: MISMATCH.
    auto* header = reinterpret_cast<PageHeader*>(storage.data());
    header->region_offset = 0u;
    TestView view(storage.data(), storage.size());
    TEST_ASSERT(view.Check() == PageMagicStatus::MISMATCH);
  }
}

}  // namespace

int main()
{
  TestReadBack();
  TestChannels();
  TestWriteCommand();
  TestStates();
  return 0;
}
