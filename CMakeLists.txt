cmake_minimum_required(VERSION 3.16)

# shared-page-xr: SG2002 大小核共享页传输（与 libcvimpp 同层，不是 xrobot Module）。
# shared-page-xr: the SG2002 inter-core shared-page transport, at the same layer
# as libcvimpp and deliberately not an xrobot Module (no MANIFEST, no
# User/xrobot.yaml entry, never constructed by xrobot_gen_main).
#
# INTERFACE target：无平台分支，`LIBXR_SYSTEM` 不出场。页契约本身平台中性
# （`shared_page.hpp`，C606 直接用），Linux 侧的 Topic 适配是
# `linux_shared_page.hpp`，只依赖 LibXR 的 `Topic`。
# INTERFACE target: no platform branch and no `LIBXR_SYSTEM` involvement. The page
# contract itself is platform neutral (`shared_page.hpp`, used directly by C606)
# and the Linux-side topic adapter is `linux_shared_page.hpp`, which only depends
# on the LibXR `Topic`.
if(NOT TARGET shared_page_xr)
  add_library(shared_page_xr INTERFACE)
  target_include_directories(shared_page_xr INTERFACE ${CMAKE_CURRENT_LIST_DIR})
  target_compile_features(shared_page_xr INTERFACE cxx_std_20)
  target_link_libraries(shared_page_xr INTERFACE xr)
endif()
