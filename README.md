# shared-page-xr

SG2002 大小核（C606L Linux ↔ C606 RTOS）共享页传输。页是传输，Topic 仍是模块边界：
契约本身平台中性（C606 直接用），Linux 侧的 Topic 适配只依赖 LibXR 的 `Topic`。

本仓库是 `bsp-guidance-vision` / action 仓库的 submodule（两仓同 URL 同 pin，对齐
`libcvimpp` 的做法），**不是 xrobot Module**：没有 MANIFEST、不进 `User/xrobot.yaml`、
不被 `xrobot_gen_main` 构造。

契约定稿见 `bsp-guidance-vision/docs/inter-core-protocol.md`。

## 组成

```text
shared_page.hpp        SharedPage：页契约（Sample / Region / AccessUnit）+ 发布索引
                       + Latest() / Since() / Region() / Acquire()
                       C606 控制环直接用
linux_shared_page.hpp  LinuxSharedPage：drain 遥测区间 → Topic 广播；写参考区
CMakeLists.txt         INTERFACE target shared_page_xr，链接 libxr，无平台分支
```

两侧的代码形状各自最自然：

```cpp
// C606 侧控制环：SharedPage 直接用，没有 Topic，没有适配层
auto& page = /* 链接脚本分配的 SharedPage */;
page.WriteSample(sample);                  // 写 ring + 推 head
Region region = {};
if (page.Region().Read(&region) == RegionScan::CURRENT) { /* 用 region.payload */ }

// Linux 侧（RazverMaster / Recorder）：订 topic，与 camera_image 同构
SharedPage page(/* yaml 配置注入的 /dev/mem 非缓存映射 */);
LibXR::Topic topic(LibXR::Topic::CreateTopic<TelemetryBatch>("telemetry"));
LibXR::LinuxSharedPage adapter(page, topic);
adapter.Poll();                            // 1kHz：drain 新区间并发一组
```

## 页布局（4 KiB，两侧均非缓存映射）

| 偏移 | 内容 |
|---|---|
| `0` | `PageHeader`：`magic = "SharedP1"`、`page_size = 4096` |
| `8` | `TelemetryRing`：`Sample ring[64]`（2560B）+ `head` + 留白 |
| `4064` | `Region`：`RegionPayload`（24B 字段）+ 4B 留白 + `seq` |

`head` 是**已发布条数**（单调，不回绕）；物理槽位 `head % 64`；最新采样
`ring[(head-1) % 64]`；区间是半开区间 `(last_seen, head]`；`head - last_seen > 64`
时整段丢弃并记 gap（有界历史的固有竞态，详见契约第 3 节）。

`Sample` 36B（8 对齐后 40B），单位是**硬件原生宽度**：IMU 为传感器原始 LSB、舵机为
硬件命令字、tick 为 `rdtime` 原始计数。满量程刻度属 action 仓库的 IMU 驱动配置，
不焊进本契约。

`Region` 的 4B 尾部留白是契约的一部分：`seq` 必须落在 28，而 `RegionPayload` 的字段
只占 24B（最后一个 `float` 结束于 24，ABI 不补尾）。把 payload 直接做成 28B 会让
`Region` 变成 36B、`seq` 落到 32，反而违约。

## 同步原语

与 `libxr/src/structure/queue/spsc_queue_base.hpp` 的 `head_`/`tail_` 逐条对应，
不新造范式：写者先写 payload 字段、再 `head + 1`（release store）；读者读 `head`
（acquire）→ 拷 payload → 复读比对，不一致重试。非缓存映射消除 cache 维护，
acquire/release 消除访存次序问题，两者正交，都要。

不需要锁、奇偶 seqlock、futex、描述符队列，也不需要 dcache clean/invalidate。

## 访问单元页（独立映射）

遥测/参考页之外还有一页 `AccessUnitPage`：单槽「最新帧」语义，magic/CRC/借还语义
从 `camera_mailbox` 收进契约，命名空间单例消失。`Publish()` 填 payload 后 release
bump `seq`；`Acquire()` acquire 比对后返回只读视图并清 `ready`，读者慢时写者直接
覆写旧帧（丢帧而不阻塞），与 keep-latest 订阅一致。Linux 侧的自留缓冲区按此发布，
C606 侧用 `Acquire()` 取。

## 约束

* `static_assert` 把字段偏移与总长钉死。**契约无版本号**：变更双端同步，加字段永远
  是廉价操作，改已有字段偏移不是。
* 页地址不进模块构造参数：Linux 侧经 yaml 配置注入，C606 侧由链接脚本分配。
* `linux_shared_page.hpp` 是唯一依赖 LibXR `Topic` 的部分；`shared_page.hpp` 只依赖
  `libxr_def.hpp` 的 `ASSERT`，C606 侧可单独使用。

## 测试

测试放在**本仓库**的 `test/` 下：与被测头同仓（本契约有两个消费者，测试不能只属于其中
一个），检查用 `TEST_ASSERT`（始终生效，不看产品断言开关）。页由匿名映射提供，不需要
`/dev/mem`、root 或第二个进程，所以契约在主机上就能验证。

```text
test/CMakeLists.txt                测试目标与 CTest 登记
test/test_assert.hpp               始终生效的 TEST_ASSERT（与 libxr 同名同义）
test/sample.hpp                    两个测试共用的 Sample 构造与比较
test/test_shared_page.cpp          页契约与发布索引
test/test_linux_shared_page.cpp    适配器 + 真实 LibXR Topic
```

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

`test_shared_page.cpp`：布局与发布索引——`Sample`/`Region` 的字段偏移与总长、`Latest` /
`Since` 的边界（64 槽余量、gap、新纪元）、region 稳定读与撕裂重试、访问单元页的借还
语义。`test_linux_shared_page.cpp`：驱动真实 LibXR `Topic`——节律门、一组遥测进回调
订阅者、gap 后重新同步、参考/命令经独立页视图回读。

### 已知待办

`libxr` 尚未作为本仓库的 submodule pin 住（`.gitmodules` 里没有它），所以「单独构建
本仓库」目前要求旁边已有一份 libxr，或在 `bsp-guidance-vision` 里构建。补上后单独
`git clone --recursive` 即可自带测试依赖。

