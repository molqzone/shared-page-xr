/**
 * @file test_coherency.cpp
 * @brief `AccessWindow` 的意图到动作映射 / Intent-to-action mapping of `AccessWindow`.
 *
 * 这组测试不碰硬件：把一组记录型函数当作平台的 `Coherent` provider，检查页引擎在每个
 * 读写路径上到底请求了哪些维护、按什么顺序、覆盖哪些字节。它要钉住的三件事：
 *   1. 意图没有走样——写路径绝不请求 invalidate，读路径绝不请求 clean；
 *   2. 顺序正确——样本先于 `head` 可见，payload 先于 `seq` 可见；
 *   3. 范围精确——发布的是引擎真正改过的字节，不是一个整页。
 *   通道表也一样：通告是 EXCHANGE（进入同步、离开发布），查找是 READ（只 invalidate），
 *   调用方不需要认识缓存操作。
 *   The channel table is no exception: announcing is EXCHANGE (sync in, publish out) and
 *   looking up is READ (invalidate only), with the caller never touching a cache op.
 *
 * 这条测试之所以可能，正是因为维护动作现在是注入的 provider 而不是产品代码里的手写调用：
 * 在没有板子的主机上也能验证“维护不会被漏掉”。
 * These tests need no hardware: a recording set of functions plays the platform's
 * `Coherent` provider, and the page engine is checked for which maintenance it requests
 * on each path, in which order, over which bytes. The three properties pinned here:
 *   1. intent is preserved -- the write path never asks for invalidate, the read path
 *      never asks for clean;
 *   2. order is right -- the sample reaches the peer before `head`, the payload before
 *      `seq`;
 *   3. ranges are exact -- what is published is what the engine actually modified, not
 *      a whole page.
 *
 * This test exists at all because maintenance is now an injected provider rather than
 * hand-written calls in product code: "it cannot be forgotten" is verifiable on a host.
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "sample.hpp"
#include "shared_page.hpp"
// 自定义策略（记录型）不在库的显式实例化列表里，所以本 TU 自己实例化引擎。
// The recording provider is not in the library's explicit-instantiation list, so this TU
// instantiates the engine itself.
#include "shared_page_impl.hpp"
#include "test_assert.hpp"

namespace
{
using LibXRTest::Format;
using LibXRTest::make_sample;
using LibXRTest::Sample;

/// @brief 测试自有的区域载荷。The test's own region payload.
struct TestPayload
{
  uint32_t magic = 0;
  uint16_t kind = 0;
  uint16_t seq16 = 0;
  uint8_t bytes[16] = {};
};

static_assert(sizeof(TestPayload) == 24);
static_assert(sizeof(TestPayload) % sizeof(uint32_t) == 0);

enum class Call : uint8_t
{
  PUBLISH,
  REFRESH,
  SYNC,
};

struct Record
{
  Call call;
  const void* address;
  size_t bytes;
};

constexpr size_t LOG_CAPACITY = 32;
std::array<Record, LOG_CAPACITY> g_log{};
size_t g_count = 0;

void RecordOne(Call call, const void* address, size_t bytes)
{
  TEST_ASSERT(g_count < LOG_CAPACITY);
  g_log[g_count++] = Record{call, address, bytes};
}

void RecordPublish(void* address, size_t bytes)
{
  RecordOne(Call::PUBLISH, address, bytes);
}

void RecordRefresh(const void* address, size_t bytes)
{
  RecordOne(Call::REFRESH, address, bytes);
}

void RecordSync(void* address, size_t bytes) { RecordOne(Call::SYNC, address, bytes); }

/// @brief 记录型策略：三个动作都记账，然后什么也不做。
///        A recording provider: all three actions journal the call and do nothing else.
struct Recording
{
  void Publish(void* address, size_t bytes) const { RecordPublish(address, bytes); }
  void Refresh(const void* address, size_t bytes) const { RecordRefresh(address, bytes); }
  void Sync(void* address, size_t bytes) const { RecordSync(address, bytes); }
};

/// @brief 用记录型策略实例化的页：维护动作在主机上可断言。
///        The page instantiated with the recording provider: maintenance is assertable on
///        a host.
using TestPage = LibXR::SharedPage<Format, Sample, TestPayload, 0, Recording>;

/// @brief 页缓冲：页内偏移按 64B 对齐，便于断言“碰的是哪一行”。
///        Page buffer, 64B aligned so the assertions can talk about lines.
alignas(64) std::array<uint8_t, Format::PAGE_SIZE> g_storage{};

uint8_t* PageBase() { return g_storage.data(); }

uint8_t* TelemetryHead()
{
  return PageBase() + LibXR::telemetry_offset() +
         static_cast<size_t>(sizeof(Sample)) * Format::SLOT_COUNT;
}

uint8_t* RegionPayload() { return PageBase() + TestPage::Layout().region_offset; }

uint8_t* RegionState() { return RegionPayload() + TestPage::Layout().payload_size; }


uint8_t* ChannelTableBase()
{
  return PageBase() + LibXR::channel_offset(TestPage::Layout());
}

/// @brief 每次测试从一页干净的、已格式化的页开始。
///        Every test starts from one clean, formatted page.
void Prepare(TestPage& page)
{
  page.Format();
  g_count = 0;
}

bool IsWithin(const void* address, const uint8_t* begin, size_t bytes)
{
  const auto* value = static_cast<const uint8_t*>(address);
  return value >= begin && value < begin + bytes;
}

/// 1. 发布一条采样：EXCHANGE 进入时同步计数，槽位先发布，计数最后发布。
///    1. Publishing one sample: counters synced on entry, slot published first, the
///    counters published last.
void test_publish_publishes_slot_before_head()
{
  TestPage page(PageBase(), Recording{});
  Prepare(page);

  TEST_ASSERT(page.WriteSample(make_sample(1)) == 1);
  TEST_ASSERT(g_count == 3);

  // 计数器的读改写：进入时 clean+invalidate，整行都可能有过期副本。
  TEST_ASSERT(g_log[0].call == Call::SYNC);
  TEST_ASSERT(g_log[0].address == TelemetryHead());
  TEST_ASSERT(g_log[0].bytes == 2 * sizeof(uint32_t));

  // 样本槽：只在环内，且必须是完整的一个采样。
  TEST_ASSERT(g_log[1].call == Call::PUBLISH);
  TEST_ASSERT(g_log[1].bytes == sizeof(Sample));
  TEST_ASSERT(IsWithin(g_log[1].address, PageBase() + LibXR::telemetry_offset(),
                       static_cast<size_t>(sizeof(Sample)) * Format::SLOT_COUNT));

  // head 与 claim 最后一起推回，读者不会先看到 head。
  TEST_ASSERT(g_log[2].call == Call::PUBLISH);
  TEST_ASSERT(g_log[2].address == TelemetryHead());
  TEST_ASSERT(g_log[2].bytes == 2 * sizeof(uint32_t));

  // 写路径不请求 invalidate：意图没有走样。
  for (size_t index = 0; index < g_count; ++index)
  {
    TEST_ASSERT(g_log[index].call != Call::REFRESH);
  }
}

/// 2. 读最新值：三次读窗口——计数、样本、复核计数，全部是 invalidate。
///    2. Reading the latest: three read windows -- counters, sample, counters again --
///    all invalidates.
void test_read_never_publishes()
{
  TestPage page(PageBase(), Recording{});
  Prepare(page);
  TEST_ASSERT(page.WriteSample(make_sample(7)) == 1);

  g_count = 0;
  Sample sample = {};
  TEST_ASSERT(page.Latest(&sample));
  TEST_ASSERT(sample.ticks == 1007);

  TEST_ASSERT(g_count == 3);
  TEST_ASSERT(g_log[0].call == Call::REFRESH);
  TEST_ASSERT(g_log[0].address == TelemetryHead());
  TEST_ASSERT(g_log[1].call == Call::REFRESH);
  TEST_ASSERT(g_log[1].bytes == sizeof(Sample));
  TEST_ASSERT(g_log[2].call == Call::REFRESH);
  TEST_ASSERT(g_log[2].address == TelemetryHead());

  // 读路径不请求 clean：只读调用不会把脏行推给对端。
  for (size_t index = 0; index < g_count; ++index)
  {
    TEST_ASSERT(g_log[index].call != Call::PUBLISH);
  }
}

/// 3. 区域写：claim 同步 → payload 发布 → seq/claim 释放，顺序即可见性顺序。
///    3. Region write: sync the claim, publish the payload, then release seq/claim --
///    the order is the visibility order.
void test_region_write_order()
{
  TestPage page(PageBase(), Recording{});
  Prepare(page);

  TestPayload payload = {};
  payload.magic = 0x1234U;
  TEST_ASSERT(page.Region().Write(payload) == 1);

  TEST_ASSERT(g_count == 3);
  TEST_ASSERT(g_log[0].call == Call::SYNC);
  TEST_ASSERT(g_log[0].address == RegionState());
  TEST_ASSERT(g_log[0].bytes == 2 * sizeof(uint32_t));

  TEST_ASSERT(g_log[1].call == Call::PUBLISH);
  TEST_ASSERT(g_log[1].address == RegionPayload());
  TEST_ASSERT(g_log[1].bytes == sizeof(TestPayload));

  TEST_ASSERT(g_log[2].call == Call::PUBLISH);
  TEST_ASSERT(g_log[2].address == RegionState());
  TEST_ASSERT(g_log[2].bytes == 2 * sizeof(uint32_t));
}

/// 4. 破 claim 是读改写：必须先同步（丢弃可能过期的 payload/seq 副本）再推回。
///    4. Breaking a claim is a read-modify-write: sync first (drop a possibly stale
///    payload/seq copy), publish after -- never a blind store+clean.
void test_recover_syncs_before_publishing()
{
  TestPage page(PageBase(), Recording{});
  Prepare(page);

  page.RecoverStaleClaims();
  TEST_ASSERT(g_count == 4);

  // 两个 claim 各自先 SYNC，之后才出现对应的 PUBLISH。
  for (size_t index = 0; index < 4; ++index)
  {
    TEST_ASSERT(g_log[index].call == (index < 2 ? Call::SYNC : Call::PUBLISH));
  }
  for (size_t publish = 2; publish < 4; ++publish)
  {
    bool synced = false;
    for (size_t sync = 0; sync < 2; ++sync)
    {
      synced = synced || (g_log[sync].address == g_log[publish].address);
    }
    TEST_ASSERT(synced);
  }
}

/// 5. 整页格式化是一个 WRITE_SYNC 窗口：范围是整页，退出动作是 clean+invalidate。
///    5. Formatting the page is one WRITE_SYNC window over the whole page: the exit
///    action is clean+invalidate, so the page is proven to be in DRAM.
void test_format_syncs_whole_page()
{
  TestPage page(PageBase(), Recording{});
  g_count = 0;

  page.Format();
  TEST_ASSERT(g_count == 1);
  TEST_ASSERT(g_log[0].call == Call::SYNC);
  TEST_ASSERT(g_log[0].address == PageBase());
  TEST_ASSERT(g_log[0].bytes == Format::PAGE_SIZE);
}


/// 6. 通道表：通告先同步再整表发布，查找只 invalidate，范围就是那张表。
///    6. Channel table: announcing syncs then publishes the whole table, lookup only
///    invalidates, and the range is exactly that table.
void test_channel_table_windows()
{
  TestPage page(PageBase(), Recording{});
  Prepare(page);

  constexpr LibXR::ChannelAnnouncement channels[] = {
      {"alpha", 0x11U, LibXR::CHANNEL_FLAG_PRESENT},
  };
  auto table = page.Channels<2>();
  TEST_ASSERT(table.Publish(channels, 1));
  TEST_ASSERT(g_count == 2);
  TEST_ASSERT(g_log[0].call == Call::SYNC);
  TEST_ASSERT(g_log[0].address == ChannelTableBase());
  TEST_ASSERT(g_log[0].bytes == sizeof(LibXR::ChannelTable<2>));
  TEST_ASSERT(g_log[1].call == Call::PUBLISH);
  TEST_ASSERT(g_log[1].address == ChannelTableBase());
  TEST_ASSERT(g_log[1].bytes == sizeof(LibXR::ChannelTable<2>));

  g_count = 0;
  LibXR::ChannelEntry entry{};
  TEST_ASSERT(table.Find("alpha", &entry));
  TEST_ASSERT(g_count == 1);
  TEST_ASSERT(g_log[0].call == Call::REFRESH);
  TEST_ASSERT(g_log[0].address == ChannelTableBase());
  TEST_ASSERT(g_log[0].bytes == sizeof(LibXR::ChannelTable<2>));

  // 查不到名字也只看不写。
  g_count = 0;
  TEST_ASSERT(!table.Find("missing", &entry));
  TEST_ASSERT(g_count == 1);
  TEST_ASSERT(g_log[0].call == Call::REFRESH);
}

}  // namespace

int main()
{
  test_publish_publishes_slot_before_head();
  test_read_never_publishes();
  test_region_write_order();
  test_recover_syncs_before_publishing();
  test_format_syncs_whole_page();
  test_channel_table_windows();
  return 0;
}
