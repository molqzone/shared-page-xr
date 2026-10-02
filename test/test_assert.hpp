/**
 * @file test_assert.hpp
 * @brief 始终生效的测试断言 / Always-active test assertions.
 *
 * 检查失败时输出位置和表达式并终止进程，不受产品断言开关影响。与
 * `libxr/test/test_assert.hpp` 同名同义：本仓库是独立 submodule，测试要能单独构建，
 * 不能依赖被测仓库之外的测试头。
 * Print the location and expression and abort on failure, independent of product
 * assertion switches. Same name and meaning as `libxr/test/test_assert.hpp`: this
 * repository is a standalone submodule, so its tests build on their own and cannot
 * depend on a test header living outside it.
 */

#pragma once

#include <cstdio>
#include <cstdlib>

/// 测试结果检查，不受产品断言开关影响 / Test check independent of product assertions.
#define TEST_ASSERT(condition)                                                       \
  do                                                                                 \
  {                                                                                  \
    if (!(condition))                                                                \
    {                                                                                \
      std::fprintf(stderr, "%s:%d: test failed: %s\n", __FILE__, __LINE__,           \
                   #condition);                                                      \
      std::abort();                                                                  \
    }                                                                                \
  } while (0)
