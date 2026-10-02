cmake_minimum_required(VERSION 3.16)

# 只有本仓库作为顶层工程时才声明 project（与 libcvimpp 的写法一致）：作为 submodule
# 被 add_subdirectory 时沿用消费方的工程设置。
# Declare a project only when this repository is the top-level one, matching
# libcvimpp: added as a submodule it inherits the consumer's project settings.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(shared_page_xr LANGUAGES C CXX)
endif()

# shared-page-xr: SG2002 大小核共享页传输（与 libcvimpp 同层，不是 xrobot Module）。
# shared-page-xr: the SG2002 inter-core shared-page transport, at the same layer
# as libcvimpp and deliberately not an xrobot Module (no MANIFEST, no
# User/xrobot.yaml entry, never constructed by xrobot_gen_main).
#
# STATIC target（与 libxr 的默认一致）：实现放在 .cpp 里，头文件只有契约与声明。无平台
# 分支，`LIBXR_SYSTEM` 不出场。页契约本身平台中性（`shared_page.hpp`，C606 直接用），
# Linux 侧的 Topic 适配是 `linux_shared_page.hpp`，只依赖 LibXR 的 `Topic`。
# STATIC target, matching the LibXR default: implementations live in the .cpp files and
# the headers hold only the contract and the declarations. No platform branch and no
# `LIBXR_SYSTEM` involvement. The page contract itself is platform neutral
# (`shared_page.hpp`, used directly by C606) and the Linux-side topic adapter is
# `linux_shared_page.hpp`, which only depends on the LibXR `Topic`.
if(NOT TARGET shared_page_xr)
  add_library(shared_page_xr STATIC shared_page.cpp linux_shared_page.cpp)
  target_include_directories(shared_page_xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})
  target_compile_features(shared_page_xr PUBLIC cxx_std_20)
  target_link_libraries(shared_page_xr PUBLIC xr)
endif()

# 单独构建本仓库时（`cmake -S shared-page-xr -B build -DSHARED_PAGE_XR_TEST_BUILD=ON`）
# 自己带出 LibXR；作为 submodule 被消费时 `xr` 已存在，这里不重复 add_subdirectory，
# 以免把消费方的缓存配置（LIBXR_SYSTEM 等）换成默认值。
# Standalone builds (`cmake -S shared-page-xr -B build
# -DSHARED_PAGE_XR_TEST_BUILD=ON`) bring in LibXR themselves. When consumed as a
# submodule `xr` already exists and is not added again, so the consumer's cached
# configuration (LIBXR_SYSTEM and friends) is not replaced by defaults.
option(SHARED_PAGE_XR_TEST_BUILD "构建 shared-page-xr 的测试 / Build shared-page-xr tests"
       OFF)
# 必须在 add_subdirectory(test) 之前：否则 test/ 的 CTestTestfile.cmake 不会被挂到
# 顶层，ctest 会报 "No tests were found"。
# Must precede add_subdirectory(test), otherwise test/CTestTestfile.cmake is never
# pulled into the top level and ctest reports "No tests were found".
if(SHARED_PAGE_XR_TEST_BUILD)
  enable_testing()
endif()
if(SHARED_PAGE_XR_TEST_BUILD AND NOT TARGET xr)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libxr/CMakeLists.txt)
    message(FATAL_ERROR
      "SHARED_PAGE_XR_TEST_BUILD needs LibXR: init the libxr submodule or build "
      "this repo from a tree that already provides the 'xr' target")
  endif()
  set(LIBXR_SYSTEM
      linux
      CACHE STRING "LibXR system backend")
  set(LIBXR_DRIVER
      linux
      CACHE STRING "LibXR host driver backend")
  set(LIBXR_SINGLE_CORE
      ON
      CACHE BOOL "LibXR single-core build")
  add_subdirectory(libxr)
endif()

add_subdirectory(test)
