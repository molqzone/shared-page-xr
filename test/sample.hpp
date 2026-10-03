/**
 * @file sample.hpp
 * @brief 两个测试共用的 wire 类型、构造与比较 / Wire type, builder and comparison
 *        shared by both tests.
 *
 * 两个测试文件共用，所以单独一个头。`libxr/test/README.md` 说「共用辅助放最近的公共
 * 目录」，在本仓库那就是平铺的 `test/` 本身；只用一次的辅助仍留在各自测试文件里。
 * Shared by both test files, hence its own header. `libxr/test/README.md` says to put
 * shared helpers in the nearest common directory, which here is the flat `test/`
 * itself; single-use helpers still stay inside their own test file.
 *
 * `Sample` 是测试自有的 wire 采样，不对应任何产品结构：它刻意取 32B 而不是产品用过
 * 的 40B，布局检查（`test_layout`）就证明了页布局跟着外部类型走，而不是被库写死。
 * `Sample` is the test's own wire sample and corresponds to no product structure: it
 * deliberately weighs 32B rather than the 40B a product used, so the layout checks in
 * `test_layout` prove the page layout follows the injected type instead of being
 * hardcoded by the library.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "shared_page.hpp"

namespace LibXRTest
{
/** @brief 测试帧格式：刻意取 2048B 页 / 32 槽，与 `DocPageFormat` 的 4096B / 64
 *  槽不同——页几何跟着注入的 format 走，不是库常量。
 *  Test frame format: deliberately a 2048B page with 32 slots, unlike
 *  `DocPageFormat`'s 4096B/64, so the page geometry demonstrably follows the
 *  injected format rather than library constants.
 */
struct Format
{
  static constexpr uint32_t ABI_VERSION = 1;
  static constexpr size_t PAGE_SIZE = 2048;
  static constexpr uint32_t SLOT_COUNT = 32;
};

/// @brief 舵机通道数（测试采样取 2 路，产品自定）。Servo channels of the test sample.
inline constexpr uint32_t SERVO_CHANNELS = 2;

/// @brief 测试用遥测采样。Test telemetry sample.
struct Sample
{
  uint64_t ticks;
  int16_t accel[3];
  int16_t temperature;
  uint16_t servo_target[SERVO_CHANNELS];
  uint16_t servo_actual[SERVO_CHANNELS];
  uint32_t status;
};

static_assert(sizeof(Sample) == 32, "the test sample layout is pinned");
static_assert(offsetof(Sample, accel) == 8, "the test sample layout is pinned");
static_assert(offsetof(Sample, temperature) == 14, "the test sample layout is pinned");
static_assert(offsetof(Sample, servo_target) == 16, "the test sample layout is pinned");
static_assert(offsetof(Sample, servo_actual) == 20, "the test sample layout is pinned");
static_assert(offsetof(Sample, status) == 24, "the test sample layout is pinned");

/**
 * @brief 构造一条可完全复现的采样。Build one fully reproducible sample.
 *
 * 每个字段都由 `index` 决定，所以「读到的是第几条」可以逐字段核对，而不只是「不是
 * 零」。索引同时写进舵机字，通道数变化时这里和 static_assert 一起暴露问题。
 * Every field is a function of `index`, so a read can be checked field by field rather
 * than merely "not zero". The index also fills the servo words, so a channel-count
 * change shows up here together with the static_assert.
 */
inline Sample make_sample(uint32_t index)
{
  Sample sample = {};
  sample.ticks = 1000 + index;
  sample.accel[0] = static_cast<int16_t>(100 + index);
  sample.accel[1] = static_cast<int16_t>(-static_cast<int16_t>(index));
  sample.accel[2] = 300;
  sample.temperature = static_cast<int16_t>(-500 + static_cast<int16_t>(index));
  for (uint32_t channel = 0; channel < SERVO_CHANNELS; ++channel)
  {
    sample.servo_target[channel] =
        static_cast<uint16_t>(index * SERVO_CHANNELS + channel);
    sample.servo_actual[channel] =
        static_cast<uint16_t>(index * SERVO_CHANNELS + channel + 5000);
  }
  sample.status = 0xF00D0000U + index;
  return sample;
}

/**
 * @brief 逐字节比较两条采样。Compare two samples byte for byte.
 *
 * 用 `memcmp` 而不是逐字段比较：结构体的尾部留白也是契约的一部分，逐字段比较会漏掉
 * 它。Uses `memcmp` rather than field-by-field comparison: the struct's tail padding is
 * part of the contract and a field-wise check would miss it.
 */
inline bool same_sample(const Sample& lhs, const Sample& rhs)
{
  return std::memcmp(&lhs, &rhs, sizeof(Sample)) == 0;
}

}  // namespace LibXRTest
