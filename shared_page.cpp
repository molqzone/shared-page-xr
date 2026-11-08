/**
 * @file shared_page.cpp
 * @brief 页引擎的内置 provider 实例化 / The built-in provider instantiations of the page engine.
 *
 * 引擎定义在私有实现头 `shared_page_impl.hpp`（模板化）。这里只为库里自带的两个 provider
 * 显式实例化，让只用它们的消费方（Linux 工具、主机测试）不必 include 实现头；
 * 平台自定义的 provider（固件的 `THeadDcacheCoherence`）在自己的构建里 include 实现头，于是
 * 三个维护动作是静态调用、空策略被完全内联。
 *
 * The engine lives in the private implementation header `shared_page_impl.hpp` (templated).
 * Only the two built-in providers are instantiated here, so consumers that use them (the
 * Linux tools, the host tests) need not include the implementation header; a platform
 * provider (`THeadDcacheCoherence` in the firmware) includes it in its own build, making the
 * three maintenance actions static calls with an empty provider inlined away.
 */

#include "shared_page_impl.hpp"

namespace LibXR
{
template class TelemetryCore<FunctionPointerCoherence>;
template class ReferenceCore<FunctionPointerCoherence>;
template class PageCore<FunctionPointerCoherence>;

template class TelemetryCore<NoCoherence>;
template class ReferenceCore<NoCoherence>;
template class PageCore<NoCoherence>;
}  // namespace LibXR
