# shared-page-xr 的测试 / Tests for shared-page-xr
#
# 平铺在 test/ 下，一个源文件一个测试可执行文件。本仓库的头文件与测试文件都还少，
# 不值得按 `libxr/test/automatic` 那样「一个接口一层目录」镜像源码树；等同一接口长到
# 多个测试文件、或平铺到不便查找时再拆，那时目录名仍取被测源文件名。
# Flat under test/, one test executable per source file. This repository still has few
# headers and test files, which does not justify mirroring the source tree one directory
# per interface the way `libxr/test/automatic` does. Split it when one interface grows
# several test files, or when the flat list stops being scannable; the directory then
# takes the name of the source file under test.
#
# 检查用 `test_assert.hpp` 的 `TEST_ASSERT`（始终生效，不看产品断言开关），与被测头
# 同仓，风格与 libxr 的测试一致。
# Checks use `TEST_ASSERT` from `test_assert.hpp` (always active, independent of the
# product assertion switches), in the same repository as the headers under test.
#
# 只有显式打开 `SHARED_PAGE_XR_TEST_BUILD` 才会进入本目录：本仓库是 submodule，
# 单独构建时不能默认去构建测试，消费方（bsp-guidance-vision / action）在自己开启
# 测试时把该选项打开。
# This directory is entered only when `SHARED_PAGE_XR_TEST_BUILD` is ON: the repo is
# a submodule, so a standalone build must not build tests by default; a consumer
# (bsp-guidance-vision / action) turns the option on when it wants them.
if(NOT SHARED_PAGE_XR_TEST_BUILD)
  return()
endif()

if(NOT TARGET xr)
  message(FATAL_ERROR "shared-page-xr tests need the LibXR target 'xr'.")
endif()

# `shared_page.hpp`：页契约与发布索引，只依赖 libxr_def.hpp 的 ASSERT。
# `shared_page.hpp`: the page contract and publish index, depending only on ASSERT
# from libxr_def.hpp.
add_executable(shared_page_test test_shared_page.cpp)
target_link_libraries(shared_page_test PRIVATE shared_page_xr)
target_include_directories(shared_page_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(shared_page_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME shared_page_contract COMMAND shared_page_test)
set_tests_properties(shared_page_contract PROPERTIES TIMEOUT 30)

# `host_shared_page.hpp`：drain 遥测进真实 LibXR topic、参考/命令回写。
# `host_shared_page.hpp`: drain telemetry into a real LibXR topic and write the
# reference/command region back.
add_executable(host_shared_page_test test_host_shared_page.cpp)
target_link_libraries(host_shared_page_test PRIVATE shared_page_xr)
target_include_directories(host_shared_page_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(host_shared_page_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME host_shared_page_topic COMMAND host_shared_page_test)
set_tests_properties(host_shared_page_topic PROPERTIES TIMEOUT 30)
