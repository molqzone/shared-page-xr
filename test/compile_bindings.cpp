/**
 * @file compile_bindings.cpp
 * @brief 只为编译：把平台绑定头拉进一个翻译单元，让重命名和签名漂移在 CI 里报错。
 *        Compile-only: pull the platform binding header into one translation unit so a
 *        rename or a signature drift fails in CI instead of rotting silently.
 *
 * `linux_mapping.hpp` 是 Linux 侧的 UIO 绑定，**目前没有消费者**：两个仓里都没有
 * 文件 include 它，`shared_page.hpp` / `doorbell.hpp` / `command_channel.hpp` 也不引用
 * 它。没有消费者就意味着没有编译单元验证过它——上一轮把 `C906CachePolicy` 改成
 * `THeadDcacheCoherence` 时，这个文件里的改动没有被任何构建看到。
 *
 * `linux_mapping.hpp` is the Linux-side UIO binding and **has no consumer today**:
 * nothing in either repository includes it, and `shared_page.hpp` / `doorbell.hpp` /
 * `command_channel.hpp` do not reference it. No consumer means no translation unit ever
 * proves it compiles -- the last rename from `C906CachePolicy` to
 * `THeadDcacheCoherence` changed this file without a build ever looking at it.
 *
 * 这里只验编译与链接，不验行为：UIO 的 mmap 索引与 `pgprot_noncached` 语义只能在板上
 * 或在一台有 UIO 设备的主机上验。等真正的用户态消费者出现，这个文件就该被它替代。
 * This checks compilation and linking only, never behaviour: the UIO mmap indices and
 * the `pgprot_noncached` semantics can only be verified on the board or on a host with
 * a real UIO device.  When the user-space consumer lands, it replaces this file.
 */
#include "linux_mapping.hpp"

int main()
{
  return 0;
}
