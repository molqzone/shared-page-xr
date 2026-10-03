# shared-page-xr

SG2002 大小核（Linux 大核 ↔ C906L RTOS 小核）共享页传输。`SharedPage` 继承 LibXR 的
`Topic`，模块直接以 Topic 作为边界；Linux 侧扩展负责 drain。

本仓库是 `bsp-guidance-vision` / action 仓库的 submodule（两仓同 URL 同 pin，对齐
`libcvimpp` 的做法），**不是 xrobot Module**：没有 MANIFEST、不进 `User/xrobot.yaml`、
不被 `xrobot_gen_main` 构造。

契约定稿见 `bsp-guidance-vision/docs/inter-core-protocol.md`。

## 组成

```text
shared_page.hpp / .cpp        页协议 + typed 视图：SharedPage<T, P, TAG> : Topic、
                              Telemetry<T>、Reference<P>。C906L 控制环直接用
linux_shared_page.hpp / .cpp  LinuxSharedPage<T, P, TAG> : SharedPage：/dev/mem 映射 +
                              drain → Topic
CMakeLists.txt                STATIC target shared_page_xr，链接 libxr；Linux 侧只在 Linux
                              构建中编译
```

声明与实现分开：`.hpp` 保留契约与类声明，协议逻辑在 `.cpp` 的类型擦除引擎里（`detail::`），
typed 层只是带 `static_assert` 的薄转发；wire 类型变多也不会复制协议实现。

## wire 类型由应用注入

本库只拥有**协议**：4 KiB 页、页头、64 槽遥测环、`head`/`seq`/`write_state` 的同步语义
与 gap 规则。穿过页面的两个数据结构由应用以 POD 类型在**编译期**注入：

```cpp
// 产品自己的契约头，双端 include 同一份
struct Sample { uint64_t ticks; ... };   // 遥测采样，产品自定字段
struct Command { ... };                  // 参考区 payload，产品自定字段

using Page = LibXR::SharedPage<Sample, Command, WIRE_TAG>;
```

* 类型约束（`PagePod`）：trivially copyable + standard layout；总长是 4 的倍数
  （逐字原子拷贝）。字段偏移与语义是**应用的契约**，用应用侧 `static_assert` 钉住。
* 布局检查在**编译期**：环与区不重叠、`head`/`write_state`/`seq` 紧贴无填充、区落在
  页尾、槽位 4B 对齐，全部由 `MakeLayout<T, P, TAG>()` 断言。
* 双端互验在**初始化期**：`Format()` 把布局指纹（`sizeof`/`alignof`/槽位/页长/TAG 的
  FNV-1a）写进页头，`Check()` 比对，对不上返回 `MISMATCH`，绝不静默错读。字段语义变了
  而尺寸没变时，递增 `TAG` 让指纹跟着变。
* `TAG` 是产品侧的 wire 版本号；不传默认 0。

两侧的代码形状各自最自然：

```cpp
// C906L 侧控制环：SharedPage 直接用
auto& page = /* 链接脚本分配的 Page */;
page.WriteSample(sample);                        // 写 ring + 推 head
Command command = {};
if (page.Region().Read(&command, &seq) == ErrorCode::OK) { /* 用 command */ }

// Linux 侧 producer / recorder：LinuxSharedPage owns the /dev/mem mapping
LibXR::LinuxSharedPage<Sample, Command> page(/* shared-page physical address */,
                                             "telemetry", 1000);
LibXR::Topic& telemetry = page;                  // module boundary
page.Poll();                                     // 1kHz：drain 新区间发一组
```

## 页布局（4 KiB，两侧均非缓存映射）

| 偏移 | 内容 |
|---|---|
| `0` | `PageHeader`：`magic = "SharedP1"`、`page_size = 4096`、`layout` 指纹 |
| `16` | `TelemetryRing<T>`：`T ring[64]` + `head` + 写状态 |
| `PAGE_SIZE - sizeof(Region<P>)` | `Region<P>`：`P payload` + 写状态 + `seq` |

`head` 是**已发布条数**（单调，不回绕）；物理槽位 `head % 64`；最新采样
`ring[(head-1) % 64]`；区间是半开区间 `(last_seen, head]`；`head - last_seen > 64`
时整段丢弃并记 gap（有界历史的固有竞态，详见契约第 3 节）。

## 同步原语

与 `libxr/src/structure/queue/spsc_queue_base.hpp` 的 `head_`/`tail_` 一样使用
acquire/release；padding 中的写入状态覆盖 payload 的同步窗口。读者检查状态和索引，
拷贝后复读，变化就重试。非缓存映射消除 cache 维护，acquire/release 消除访存次序
问题，两者正交。

不需要 futex、描述符队列或异步状态机，也不需要 dcache clean/invalidate。

## 约束

* **契约无独立版本号字段**：布局指纹 + 产品 `TAG` 承担互验，变更双端同步。加字段永远
  是廉价操作，改已有字段偏移不是——改了就必须递增 `TAG` 并同步另一侧。
* 页地址不进模块构造参数：Linux 侧经 yaml 配置注入，C906L 侧由链接脚本分配。
* `SharedPage` 继承 LibXR `Topic`；`LinuxSharedPage` 映射 Linux 物理页并发布 typed topic。
  两者都需要编译本仓库的 `.cpp`。

## 测试

测试放在**本仓库**的 `test/` 下：与被测头同仓（本契约有两个消费者，测试不能只属于其中
一个），检查用 `TEST_ASSERT`（始终生效，不看产品断言开关）。页由匿名映射提供，不需要
`/dev/mem`、root 或第二个进程，所以契约在主机上就能验证。

```text
test/CMakeLists.txt                测试目标与 CTest 登记
test/test_assert.hpp               始终生效的 TEST_ASSERT（与 libxr 同名同义）
test/sample.hpp                    测试自有的 wire 类型、构造与比较
test/test_shared_page.cpp          页契约与发布索引
test/test_linux_shared_page.cpp    适配器 + 真实 LibXR Topic
```

测试用的 `Sample` 是 32B 的测试类型，不对应任何产品结构：它与产品尺寸不同，布局检查
就证明了页布局跟着注入类型走，而不是被库写死。

**平铺而不是一个接口一层目录**：本仓库只有两个头、两个测试文件，`libxr/test/automatic`
那种镜像源码树的层级在这里只是额外跳转。等同一接口长到多个测试文件再拆，那时目录名取
被测源文件名（比如 `test/shared_page/`）。

测试**默认不构建**（本仓库是 submodule，单独构建时不该带出测试），显式打开：

```bash
# 单独构建本仓库（需要 libxr；见下方“已知待办”）
cmake -S . -B build -DSHARED_PAGE_XR_TEST_BUILD=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

消费方在自己开启测试时把该选项打开，例如 `bsp-guidance-vision`：

```bash
cmake --preset debug && cmake --build --preset debug
ctest --test-dir build-host -R "shared_page|linux_shared_page" --output-on-failure
```

`test_shared_page.cpp`：布局与发布索引——`TelemetryRing`/`Region` 跟随注入类型的偏移
与总长、`PageLayout` 摘要与 `MISMATCH` 互验、`Latest`/`Since` 的边界（64 槽余量、gap、
新纪元）、region 稳定读与撕裂重试。
`test_linux_shared_page.cpp`：驱动真实 LibXR `Topic`——节律门、一组遥测进回调订阅者、
gap 后重新同步、以及由高层 payload 写入的参考区回读。

### 已知待办

`libxr` 尚未作为本仓库的 submodule pin 住（`.gitmodules` 里没有它），所以「单独构建
本仓库」目前要求旁边已有一份 libxr，或在 `bsp-guidance-vision` 里构建。补上后单独
`git clone --recursive` 即可自带测试依赖。
