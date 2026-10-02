/**
 * @file sample.hpp
 * @brief 两个测试共用的 Sample 构造与比较 / Sample builder and comparison shared by
 *        both tests.
 *
 * 两个测试文件共用，所以单独一个头。`libxr/test/README.md` 说「共用辅助放最近的公共
 * 目录」，在本仓库那就是平铺的 `test/` 本身；只用一次的辅助仍留在各自测试文件里。
 * Shared by both test files, hence its own header. `libxr/test/README.md` says to put
 * shared helpers in the nearest common directory, which here is the flat `test/`
 * itself; single-use helpers still stay inside their own test file.
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "shared_page.hpp"

namespace SharedPageXRTest
{
/// @brief 契约里的舵机通道数（第 8 节定稿）。Servo channel count from the settled §8.
inline constexpr uint32_t SERVO_CHANNELS = 4;

/**
 * @brief 构造一条可完全复现的采样。Build one fully reproducible sample.
 *
 * 每个字段都由 `index` 决定，所以「读到的是第几条」可以逐字段核对，而不只是「不是
 * 零」。索引同时写进 4 路舵机字，通道数变化时这里和 static_assert 一起暴露问题。
 * Every field is a function of `index`, so a read can be checked field by field rather
 * than merely "not zero". The index also fills the four servo words, so a channel-count
 * change shows up here together with the static_assert.
 */
inline SharedPageXR::Sample MakeSample(uint32_t index)
{
  SharedPageXR::Sample sample = {};
  sample.ticks = 1000 + index;
  sample.accel[0] = static_cast<int16_t>(100 + index);
  sample.accel[1] = static_cast<int16_t>(-static_cast<int16_t>(index));
  sample.accel[2] = 300;
  sample.gyro[0] = static_cast<int16_t>(-static_cast<int16_t>(index));
  sample.gyro[1] = static_cast<int16_t>(200 + index);
  sample.gyro[2] = -400;
  for (uint32_t channel = 0; channel < SERVO_CHANNELS; ++channel)
  {
    sample.servo_target[channel] =
        static_cast<uint16_t>(index * SERVO_CHANNELS + channel);
  }
  sample.pad = 0xDEADBEEFU;
  return sample;
}

/**
 * @brief 逐字节比较两条采样。Compare two samples byte for byte.
 *
 * 用 `memcmp` 而不是逐字段比较：结构体的 4B 尾部留白也是契约的一部分，逐字段比较会
 * 漏掉它。
 * Uses `memcmp` rather than field-by-field comparison: the struct's 4B tail padding is
 * part of the contract and a field-wise check would miss it.
 */
inline bool SameSample(const SharedPageXR::Sample& lhs, const SharedPageXR::Sample& rhs)
{
  return std::memcmp(&lhs, &rhs, sizeof(SharedPageXR::Sample)) == 0;
}

}  // namespace SharedPageXRTest
