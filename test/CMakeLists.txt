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

# `AccessWindow`：意图到动作的映射——写路径不请求 invalidate、读路径不请求 clean、
# 范围就是真正改过的字节。用记录型策略在主机上验证“维护不会被漏掉”。
# `AccessWindow`: intent-to-action mapping -- the write path never asks for
# invalidate, the read path never for clean, and the ranges are the bytes actually
# modified. A recording policy proves on a host that maintenance cannot be skipped.
add_executable(coherency_test test_coherency.cpp)
target_link_libraries(coherency_test PRIVATE shared_page_xr)
target_include_directories(coherency_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(coherency_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME coherency_window COMMAND coherency_test)
set_tests_properties(coherency_window PROPERTIES TIMEOUT 30)

# `doorbell.hpp`：三个动词各走各的指针——`Signal`/`Reply` 在这块硬件上共用一套 mailbox
# 寄存器，混淆了在板上会表现成“遥测把回执吃掉”，所以在主机上先钉住。
# `doorbell.hpp`: each verb reaches its own pointer -- `Signal` and `Reply` share one set
# of mailbox registers on this hardware, so mixing them up would show up on the board as
# "telemetry ate the reply". Pinned on a host first.
add_executable(doorbell_test test_doorbell.cpp)
target_link_libraries(doorbell_test PRIVATE shared_page_xr)
target_include_directories(doorbell_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(doorbell_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME doorbell_seam COMMAND doorbell_test)
set_tests_properties(doorbell_seam PROPERTIES TIMEOUT 30)

# `command_channel.hpp`：命令方向的四件事（通知、区域快照、校验、重复通知重放）。
# 通知侧是假的 `Doorbell`，所以“有没有越权回执”也能在主机上钉住。
# `command_channel.hpp`: the four command-direction steps (notify, region snapshot,
# validate, replay on a repeated notification). The notification side is a fake
# `Doorbell`, so "did it reply when nothing was outstanding" is pinned on a host too.
add_executable(command_channel_test test_command_channel.cpp)
target_link_libraries(command_channel_test PRIVATE shared_page_xr)
target_include_directories(command_channel_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(command_channel_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME command_channel_contract COMMAND command_channel_test)
set_tests_properties(command_channel_contract PROPERTIES TIMEOUT 30)
