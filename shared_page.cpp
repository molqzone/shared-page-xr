/**
 * @file shared_page.cpp
 * @brief 页引擎的内置策略实例化 / The built-in policy instantiations of the page engine.
 *
 * 引擎定义在私有实现头 `shared_page_impl.hpp`（模板化）。这里只为库里自带的两个策略
 * 显式实例化，让只用它们的消费方（Linux 工具、主机测试）不必 include 实现头；
 * 平台自定义的策略（固件的 `C906CachePolicy`）在自己的构建里 include 实现头，于是
 * 三个维护动作是静态调用、空策略被完全内联。
 *
 * The engine lives in the private implementation header `shared_page_impl.hpp` (templated).
 * Only the two built-in policies are instantiated here, so consumers that use them (the
 * Linux tools, the host tests) need not include the implementation header; a platform
 * policy (`C906CachePolicy` in the firmware) includes it in its own build, making the
 * three maintenance actions static calls with an empty policy inlined away.
 */

#include "shared_page_impl.hpp"

namespace LibXR
{
template class TelemetryCore<FunctionPointerPolicy>;
template class ReferenceCore<FunctionPointerPolicy>;
template class PageCore<FunctionPointerPolicy>;

template class TelemetryCore<NoCachePolicy>;
template class ReferenceCore<NoCachePolicy>;
template class PageCore<NoCachePolicy>;
}  // namespace LibXR
