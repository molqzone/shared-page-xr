/**
 * @file test_linux_mapping.cpp
 * @brief uio 的 sysfs 发现：设备名 → 目录名、map 名 → 索引与字节数。
 *        uio discovery through sysfs: device name -> directory, map name -> index and
 *        size.
 *
 * 真机上的 `/sys/class/uio` 当然造不出来，但这三个函数都收一个 `root`，所以这里搭一棵
 * 假树，把**选择逻辑**钉住：一个节点有两段 `reg` 时，选中的必须是 `name` 为 `shared`
 * 的那一段，字节数必须来自 `size`——而不是"index 1 恰好是对的"。
 *
 * A real `/sys/class/uio` cannot be faked into existence, but all three helpers take a
 * `root`, so this builds a fake tree and pins the **selection** logic: with two `reg`
 * entries on one node the chosen one must be the entry named `shared`, and its byte
 * count must come from `size` -- not from "index 1 happening to be right".
 *
 * 为什么值得钉：`bsp-guidance-vision` 先手写了一遍这份查找，库这份又写死 `map 1`；
 * 两份对同一棵 sysfs 树给出不同答案时，错的会表现成"读到了 console 页"，而
 * `pgprot_noncached` 让这种错读不报任何错。
 *
 * Why it is worth pinning: `bsp-guidance-vision` hand-wrote this lookup once and the
 * library once hard-coded `map 1`; when the two give different answers for the same
 * sysfs tree, the wrong one reads the console page -- and `pgprot_noncached` makes that
 * misread completely silent.
 */

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include "linux_mapping.hpp"
#include "test_assert.hpp"

namespace
{
/// @brief 一棵假 sysfs 树，析构时删掉 / A fake sysfs tree, removed on destruction.
struct FakeUio
{
  std::string root;

  FakeUio()
  {
    char pattern[] = "/tmp/spxr-uio-XXXXXX";
    const char* dir = ::mkdtemp(pattern);
    TEST_ASSERT(dir != nullptr);
    root = dir;
  }

  ~FakeUio()
  {
    std::error_code ignored;
    std::filesystem::remove_all(root, ignored);
  }

  FakeUio(const FakeUio&) = delete;
  FakeUio& operator=(const FakeUio&) = delete;

  /// @brief 写一个属性文件（父目录自动创建）/ Write one attribute, creating parents.
  void Write(const std::string& path, const std::string& value)
  {
    const std::filesystem::path file = std::filesystem::path(root) / path;
    std::filesystem::create_directories(file.parent_path());
    std::ofstream out(file);
    TEST_ASSERT(out.good());
    out << value;
  }
};

/// @brief 造出真机形状的树：uio0 是别的设备，uio1 才是 guidance-pages。
///        Build the shape a real machine has: uio0 is something else, uio1 is ours.
void BuildMachineTree(FakeUio* tree)
{
  tree->Write("uio0/name", "some-other-node\n");
  tree->Write("uio1/name", "guidance-pages\n");
  tree->Write("uio1/maps/map0/name", "console\n");
  tree->Write("uio1/maps/map0/size", "8192\n");
  tree->Write("uio1/maps/map1/name", "shared\n");
  tree->Write("uio1/maps/map1/size", "4096\n");
}

void TestDeviceLookup()
{
  FakeUio tree;
  BuildMachineTree(&tree);

  // 名字匹配的是 sysfs 的 `name`，不是目录序号：uio1 才是我们的节点。
  // Match on the sysfs `name`, not on the directory number: uio1 is the one.
  TEST_ASSERT(LibXR::UIO::FindDevice(tree.root.c_str(), "guidance-pages") == "uio1");
  TEST_ASSERT(LibXR::UIO::FindDevice(tree.root.c_str(), "some-other-node") == "uio0");
  TEST_ASSERT(LibXR::UIO::FindDevice(tree.root.c_str(), "absent").empty());

  // 空 root 与空名字都只是"没找到"，不崩、不读别的设备。
  // An empty root or name is just "not found": no crash, no reading another device.
  TEST_ASSERT(LibXR::UIO::FindDevice(nullptr, "guidance-pages").empty());
  TEST_ASSERT(LibXR::UIO::FindDevice(tree.root.c_str(), nullptr).empty());
}

void TestMapSelection()
{
  FakeUio tree;
  BuildMachineTree(&tree);

  LibXR::UIO::MapInfo info;
  TEST_ASSERT(LibXR::UIO::FindMapByName(tree.root.c_str(), "uio1", "shared", &info));
  // 索引与字节数都来自 sysfs，不是写死的 1 和 4096。
  // Both the index and the size come from sysfs, not from a hard-coded 1 and 4096.
  TEST_ASSERT(info.index == 1);
  TEST_ASSERT(info.bytes == 4096);

  TEST_ASSERT(LibXR::UIO::FindMapByName(tree.root.c_str(), "uio1", "console", &info));
  TEST_ASSERT(info.index == 0);
  TEST_ASSERT(info.bytes == 8192);

  // 名字不存在时 false，不能退化成"随便给一个".
  // An unknown name is false; it never degrades to "some map will do".
  TEST_ASSERT(!LibXR::UIO::FindMapByName(tree.root.c_str(), "uio1", "absent", &info));
  TEST_ASSERT(!LibXR::UIO::FindMapByName(tree.root.c_str(), "uio9", "shared", &info));
}

void TestSizeAndTrim()
{
  FakeUio tree;
  BuildMachineTree(&tree);
  // 真机的属性文件带换行，有的还带空格：读出来必须是干净的数字/名字。
  // Real attribute files end in a newline, some with trailing spaces: what comes out
  // must be a clean number or name.
  tree.Write("uio1/maps/map2/name", "  spaced  \r\n");
  tree.Write("uio1/maps/map2/size", " 1024 \n");

  size_t bytes = 0;
  TEST_ASSERT(LibXR::UIO::ReadMapSize(tree.root.c_str(), "uio1", 1, &bytes));
  TEST_ASSERT(bytes == 4096);
  TEST_ASSERT(LibXR::UIO::ReadMapSize(tree.root.c_str(), "uio1", 2, &bytes));
  TEST_ASSERT(bytes == 1024);
  TEST_ASSERT(!LibXR::UIO::ReadMapSize(tree.root.c_str(), "uio1", 9, &bytes));

  // 前导空格不参与匹配（sysfs 不会有，但读进来的字符串也不该被它骗）。
  // Leading spaces do not take part in matching (sysfs never has them, but the string
  // that comes back should not be fooled either).
  TEST_ASSERT(LibXR::UIO::ReadAttribute(std::string(tree.root) + "/uio1/maps/map2/name")
                  .find("spaced") != std::string::npos);
}

void TestOpenRejectsBadInput()
{
  // 没有设备名或没有长度，就不该尝试 open `/dev/` 下的任何东西。
  // Without a device name or a length, nothing under `/dev/` is opened.
  TEST_ASSERT(!LibXR::UIOMapping::Open(nullptr, 0, 4096));
  TEST_ASSERT(!LibXR::UIOMapping::Open("no-such-uio-device", 1, 4096));
  TEST_ASSERT(!LibXR::UIOMapping::Open("no-such-uio-device", 1, 0));
  TEST_ASSERT(!LibXR::UIOMapping::OpenNamed(nullptr, "shared", 1, 4096));
  TEST_ASSERT(!LibXR::UIOMapping::OpenNamed("no-such-uio-device", "shared", 1, 4096));
  TEST_ASSERT(!LibXR::UIOMapping::OpenNamed("no-such-uio-device", "shared", 1, 0));
}

}  // namespace

int main()
{
  TestDeviceLookup();
  TestMapSelection();
  TestSizeAndTrim();
  TestOpenRejectsBadInput();
  std::printf("linux_mapping: discovery contracts hold\n");
  return 0;
}
