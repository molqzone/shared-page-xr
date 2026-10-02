# shared-page-xr 的测试 / Tests for shared-page-xr
#
# 与 `libxr/test/automatic` 同一组织方式：目录名是被测接口的源文件名，测试用
# `TEST_ASSERT` 检查（始终生效，不看产品断言开关）。
# Organised like `libxr/test/automatic`: the directory name is the source file under
# test and the checks use `TEST_ASSERT`, which is always active.
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
add_executable(shared_page_test ${CMAKE_CURRENT_LIST_DIR}/shared_page/test_shared_page.cpp)
target_link_libraries(shared_page_test PRIVATE shared_page_xr)
target_include_directories(shared_page_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}
                                                  ${CMAKE_CURRENT_LIST_DIR}/common)
target_compile_options(shared_page_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME shared_page_contract COMMAND shared_page_test)
set_tests_properties(shared_page_contract PROPERTIES TIMEOUT 30)

# `linux_shared_page.hpp`：drain 遥测进真实 LibXR topic、参考/命令回写。
# `linux_shared_page.hpp`: drain telemetry into a real LibXR topic and write the
# reference/command region back.
add_executable(linux_shared_page_test
               ${CMAKE_CURRENT_LIST_DIR}/linux_shared_page/test_linux_shared_page.cpp)
target_link_libraries(linux_shared_page_test PRIVATE shared_page_xr)
target_include_directories(linux_shared_page_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}
                                                        ${CMAKE_CURRENT_LIST_DIR}/common)
target_compile_options(linux_shared_page_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME linux_shared_page_topic COMMAND linux_shared_page_test)
set_tests_properties(linux_shared_page_topic PROPERTIES TIMEOUT 30)
