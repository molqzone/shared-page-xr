# shared-page-xr

SG2002 大小核（Linux 大核 ↔ C906L RTOS 小核）共享页传输。`SharedPage` 继承 LibXR 的
`Topic`，模块直接以 Topic 作为边界；宿主侧适配器负责 drain。

本仓库是 `bsp-guidance-action` 的 submodule（与 `libcvimpp` 在 vision 仓里的位置同层，
做法一致），**不是 xrobot Module**：没有 MANIFEST、不进 `User/xrobot.yaml`、不被
`xrobot_gen_main` 构造。

本仓库的传输语义（页布局、keep-latest、区域快照、claim、一致性接缝）以本 README 为准；
产品那一层的定稿规格是消费方仓的 `docs/inter-core-protocol.md`（`bsp-guidance-action`），
wire 类型（`Sample` / `Command` / 帧几何 / 通道表）在它的 `User/guidance_contract.hpp`，
并由同仓的 `tools/guidance-contract` 发布成机器可读的 `guidance_contract.json`。

## 组成

```text
shared_page.hpp / .cpp        页协议 + typed 视图：SharedPage<F, T, P, TAG, Policy> : Topic、
                               Telemetry<T, Policy>、Reference<P, Policy>、NameService；
                               一致性由**编译期的 provider** 给出（THeadDcacheCoherence / NoCoherence），
                               访问窗口 AccessWindow<Provider> 静态调用它。平台中立，两端同用
shared_page_impl.hpp          私有实现头：模板化的页引擎，shared_page.cpp 为内置 provider 显式
                               实例化；自定义 provider 的消费方 include 它来实例化自己那份
doorbell.hpp                  响铃接缝（header-only）：Doorbell = Wait（等通知，OK 即有
                               在途请求）+ Signal（无载荷叫醒对端）+ Reply（回答刚那条），
                               全是注入的函数指针，平台中立
channel table                 页布局的一部分（`ChannelTable<CAPACITY>` / `NameService`）：
                              名字 → 端点的通告与发现，代次语义，维护由页绑定的 provider 负责
command_channel.hpp           命令方向（header-only）：Doorbell（通知+回执，注入）
                               + CommandChannel<F, T, P, TAG, Policy>：等、读、校验、去重、回执
linux_mapping.hpp             Linux 绑定：UioMapping（按 sysfs 名找 /dev/uioN，按 map
                               索引 mmap）+ 该映射的一致性域（pgprot_noncached →
                               NoCoherence）。只有 Linux 消费者显式 include 才进入
                               编译，中立核心不引用它
CMakeLists.txt                STATIC target shared_page_xr，只编 shared_page.cpp，链接 libxr；
                               doorbell / command_channel / linux_mapping 是 header-only，
                               不触平台 UAPI，任何平台可编译
```

声明与实现分开：`shared_page.hpp` 保留契约与类声明，协议逻辑在私有实现头
`shared_page_impl.hpp`（模板化的 `TelemetryCore` / `ReferenceCore` / `PageCore`），
`shared_page.cpp` 只为内置 provider 显式实例化；typed 层只是带 `static_assert` 的薄转发，
所以 wire 类型变了也不会复制协议实现。

**数据路径由消费侧驱动，库不替它决定怎么等。** `SharedPage` 只是一个绑定到页的视图：
页指针与一致性域是构造参数，什么时候读、被什么唤醒由调用方决定（板端是
`RTOS_CMDQU_WAIT_EVENT` 的阻塞唤醒，见消费方仓的用户态工具）。刻意不提供轮询入口：
轮询一旦存在就会因为“方便”变成实际使用的那条。周期代码只许看状态（`Head()` 的时鲜），
不许过手数据。

本库不接触任何平台 UAPI——页指针是注入的：Linux 侧的 UIO 绑定在 `linux_mapping.hpp`，
由消费者自行 include；其他平台给链接脚本符号或静态缓冲即可。

原先的宿主适配器（等 `LibXR::Semaphore` → drain → publish）已删除：它的唯一消费者是
自己的测试，而且它不认识帧布局，会把未校验的样本 publish 出去。等真的出现第二个消费者、
形状明确时再写，并把 `Guidance::ReadSample()` 的校验一并接进去。

## 帧格式是平台数据，载荷是应用数据，语义归本库

三层各有其主：

* **帧/几何**（页多大、多少槽、什么偏移）是**平台数据**：由平台内存图与 ABI 定
  （例如 sophgo-sg200x-debian 的 `memmap.py` 划区域，
  `sg2002_rtos_shm_layout_line` 自描述页头）。编译期以 format traits 注入：库**不带任何
  SoC 的默认几何**，`PageFormat` concept 只约束 `ABI_VERSION` / `PAGE_SIZE` /
  `SLOT_COUNT` 三个成员；初始化期由**自描述页头逐项互验**——页头把每个几何数字写在页上，
  照 `sg2002-ipc` `layout_line` 的做法，`Check()` 逐项比对，任何漂移报 `MISMATCH`，
  绝不静默错读。
* **载荷**（遥测采样、参考区 payload 的字段）是**应用数据**：以 POD 类型在编译期注入。
* **同步语义**是本库的：keep-latest 遥测环、有界历史的 gap 规则、自洽区域快照、
  `head`/`seq`/`write_state` 的 acquire-release 协议。

```cpp
// 产品自己的契约头，双端 include 同一份
struct Format                                 // 平台几何：本板 4KiB/58 槽，见 guidance_contract.hpp
{
  static constexpr uint32_t ABI_VERSION = 1;
  static constexpr size_t PAGE_SIZE = 4096;
  static constexpr uint32_t SLOT_COUNT = 58;
};
struct Sample { uint64_t ticks; ... };    // 遥测采样，产品自定字段
struct Command { ... };                   // 参考区 payload，产品自定字段

using Page = LibXR::SharedPage<Format, Sample, Command, WIRE_TAG>;
```

* 类型约束（`PagePod`）：trivially copyable + standard layout；总长是 4 的倍数
  （逐字原子拷贝）。字段偏移与语义是**应用的契约**，用应用侧 `static_assert` 钉住。
* 布局检查在**编译期**：环与区不重叠、计数器紧贴无填充、区落在页尾、槽位 4B 对齐，
  全部由 `make_page_layout<F, T, P, TAG>()` 断言。
* 双端互验在**初始化期**：页头记录 abi_version/页长/槽位/双端载荷尺寸/区偏移/TAG，
  `Check()` 逐项比对。字段语义变了而尺寸没变时，递增 `TAG`；帧几何变了换 `Format`。
* `TAG` 是产品侧的 wire 版本号；不传默认 0。

两侧的代码形状各自最自然：

```cpp
// C906L 侧控制环：SharedPage 直接用
auto& page = /* 链接脚本分配的 Page */;
page.WriteSample(sample);                        // 写 ring + 推 head
Command command = {};
if (page.Region().Read(&command, &seq) == ErrorCode::OK) { /* 用 command */ }

// 消费侧（Linux / 另一核）：页指针与一致性域都是构造参数
auto mapping = LibXR::UioMapping::Open("guidance-pages", 1, 4096);  // Linux 绑定
Page page(mapping.Data(), mapping.Coherence());  // 域由映射给出，必须给
uint32_t seen = page.TelemetryReader().Head();   // 从当前 head 起，只跟新样本
LibXR::SinceResult since{};
for (;;) {
  WaitForNotification();                         // 消费侧自己的事件源（板端：
                                                 // RTOS_CMDQU_WAIT_EVENT）
  Sample batch[Format::SLOT_COUNT];
  if (page.TelemetryReader().Since(seen, batch, Format::SLOT_COUNT, &since) ==
      LibXR::ErrorCode::OK) {
    seen = since.next;                           // since.gap 即丢掉的样本数
    // `SharedPage` 继承 `Topic`：要不要把 batch 发布出去、按什么时序，由消费侧决定。
  }
}
```

## 命令方向

遥测是“页面往上给一个 topic”，命令是“页面往上给一条已验证的命令”——两个方向的内容
都由页层吃掉，产品只写业务：

```cpp
// C906L 侧：通知与回执由平台绑定实现（这里是 mailbox doorbell），页层不认识它
Guidance::CommandChannel channel(page, mailbox_port.Port(), Guidance::CommandValid);
for (;;) {
  Guidance::Command command = {};
  uint32_t seq = 0;
  if (channel.Wait(LibXR::WAIT_FOREVER, &command, &seq) != LibXR::ErrorCode::OK) {
    continue;                       // 超时/无效/重复，页层已经按合同回执过
  }
  channel.Ack(seq, ApplyCommand(command));   // 产品只剩这一行
}
```

`CommandChannel::Wait` 依次做四件事：等通知、判页可用、读区域自洽快照、校验载荷；
`Ack` 才发回执。所以失败路径（页未格式化、快照不收敛、载荷无效）也会回执
`COMMAND_REJECTED`，发送方不会因为“没人回答”而超时。

**重复通知**（doorbell 合并，或发送方没收到回执而重试）不重放命令，而是重发上一次的
回执：重放会让"点亮 LED"这类有副作用的命令执行第二次，不回执会让发送方超时。

## 页布局（页长/槽位随 format，两侧均非缓存映射）

| 偏移 | 内容 |
|---|---|
| `0` | `PageHeader`（自描述）：`magic`、`abi_version`、`page_size`、`slot_count`、`sample_size`、`payload_size`、`region_offset`、`tag` |
| `sizeof(PageHeader)` | `TelemetryRing<T, SLOTS>`：`T ring[SLOTS]` + `head` + 写状态 |
| `channel_offset(layout)` | `ChannelTable<CAPACITY>`：一代通告的头 + `CAPACITY` 个 `ChannelEntry`（名字、CRC32 键、端点、flags） |
| `PAGE_SIZE - sizeof(Region<P>)` | `Region<P>`：`P payload` + 写状态 + `seq` |

通道表是页布局的一部分，不再是产品在页尾自己划的一块：偏移由
`channel_offset(layout)` 从环的几何算出来（环的计数器之后的那条缓存行），容量由产品
声明、由 `channel_table_fits<CAPACITY>()` 在编译期复核。表头自描述（魔数/条目数/代次），
所以“还没通告”和“通告了空表”可区分；通告与查找走 `NameService<CAPACITY>`
（`SharedPage::Channels<CAPACITY>()`），调用方不需要认识缓存操作。
The channel table is part of the page layout instead of an area the product carved out at
the tail: `channel_offset(layout)` derives it from the ring geometry (the cache line after
the ring's counters), the product declares the capacity, and
`channel_table_fits<CAPACITY>()` re-checks it at compile time.  Its header describes
itself (magic/count/generation), so "never announced" and "announced as empty" stay
distinguishable, and announcing/looking up go through `NameService<CAPACITY>`
(`SharedPage::Channels<CAPACITY>()`) -- the caller never handles cache maintenance.

`head` 是**已发布条数**（单调，不回绕）；物理槽位 `head % SLOT_COUNT`；最新采样
`ring[(head-1) % SLOT_COUNT]`；区间是半开区间 `(last_seen, head]`；`head - last_seen > SLOT_COUNT`
时整段丢弃并记 gap（有界历史的固有竞态，详见契约第 3 节）。

## 同步原语

与 `libxr/src/structure/queue/spsc_queue_base.hpp` 的 `head_`/`tail_` 一样使用
acquire/release；padding 中的写入状态覆盖 payload 的同步窗口。读者检查状态和索引，
拷贝后复读，变化就重试。**协议本身**不需要 futex、描述符队列或异步状态机；
acquire/release 消除访存次序问题，可见性则交给注入的 provider，两者正交。

**可见性归 provider，不归协议。** 对端是 non-cacheable 映射（Linux 的 uio 映射）或单地址
空间时，三个动作是空的（`NoCoherence`）；像 C906 这样无法把共享页排除出可缓存区间的
平台给自己的 `Page` 别名填 `THeadDcacheCoherence`（三个动作直接调 arch 原语）。provider 是
`SharedPage` 的**模板参数**（概念 `Coherent`），所以漏写就是编译错误、
空 provider 被完全内联、真实实现没有间接调用。页引擎用 `AccessWindow<Provider>` 按意图成对调用：

| 意图 | 进入时 | 离开时 |
|---|---|---|
| `READ` | `Refresh`（invalidate） | — |
| `WRITE` | — | `Publish`（clean） |
| `EXCHANGE` | `Sync`（clean+invalidate） | `Publish`（clean） |

provider 是 `SharedPage` 的模板参数：没有“省略即不维护”的路径，因为漏写 provider 类型
根本不编译。Linux 侧把答案放在映射旁边（`mapping.Coherence()` 返回 `NoCoherence`），固件
侧在类型别名里写死（`Guidance::Page<LibXR::THeadDcacheCoherence>`）。

读到一半的槽位永远在 `head` 之外，所以读路径只需要 `Refresh`；而 claim、
`ClearHistory`、`RecoverStaleClaims` 这类读改写必须两边都动（`EXCHANGE`），旧的两钩子
接口只能表达一半，这是它被替换的直接原因。

**写者死亡**：写者死在 claim 窗口里（进程被杀/固件崩溃）会让 `write_state` 永久
占用，读者从此只拿 `BUSY`。机制是 `SharedPage::RecoverStaleClaims()`（只破 claim，
不动 `head`/`seq`/数据——未发布完的槽位在 `head` 之外，不丢历史、不漏撕裂数据）；
执行者是**带时钟的那一侧**：板端工具与固件各自从自己的健康检查里判断“持续 `BUSY` 超过
阈值 = 写者已死”，再调这个机制（阈值远大于正常 claim 窗口，例如 10ms）。

## 约束

* **契约无独立版本号字段**：页头自描述 + 产品 `TAG` 承担互验，变更双端同步。加字段永远
  是廉价操作，改已有字段偏移不是——改了就必须递增 `TAG` 并同步另一侧；改帧几何就换
  `Format` traits。
* 页地址以指针注入，不进契约：Linux 侧由平台映射后传入（参考绑定
  `linux_mapping.hpp` 按名字找 `/dev/uioN`），C906L 侧由链接脚本分配的符号传入
  （平台内存图数据，见 sophgo-sg200x-debian 的 `memmap.py`：页在 `fast_image` 这块
  保留区的尾部）。
* `SharedPage` 继承 LibXR `Topic`：页本身就能当 topic 发布出去，等事件、drain、发布
  的时序由消费侧决定（本仓库不替它做决定——原先的 `SharedPageHost` 适配器已删除，因为
  除它自己的测试外没有消费者；等 UIO + 用户态 LibXR 真接上线时，按那时的消费者形状再写，
  并把 `Guidance::ReadSample()` 的校验一并接进去）。

## 测试

测试放在**本仓库**的 `test/` 下：与被测头同仓（本契约有两个消费者，测试不能只属于其中
一个），检查用 `TEST_ASSERT`（始终生效，不看产品断言开关）。页是进程内静态缓冲、事件源
是 LibXR `Semaphore`，不需要 uio 设备、root、eventfd 或第二个进程，所以契约加适配器
在主机上就能验证。

```text
test/CMakeLists.txt                测试目标与 CTest 登记
test/test_assert.hpp               始终生效的 TEST_ASSERT（与 libxr 同名同义）
test/sample.hpp                    测试自有的 wire 类型、构造与比较
test/test_shared_page.cpp          页契约与发布索引
test/test_coherency.cpp            访问窗口：意图→动作、顺序、范围
test/test_command_channel.cpp      命令方向：校验、去重、回执合同
```

测试用的 `Sample` 是 32B 的测试类型、`Format` 是 2048B 页 / 32 槽的测试几何，均不对应
任何产品定义：它们与产品不同，布局检查就证明了帧几何与载荷都跟着注入的 traits 与类型
走，而不是被库写死。

**平铺而不是一个接口一层目录**：本仓库的头与测试文件都还少，`libxr/test/automatic`
那种镜像源码树的层级在这里只是额外跳转。等同一接口长到多个测试文件再拆，那时目录名取
被测源文件名（比如 `test/shared_page/`）。

测试**默认不构建**（本仓库是 submodule，单独构建时不该带出测试），显式打开：

```bash
# 单独构建本仓库（需要 libxr；见下方“已知待办”）
cmake -S . -B build -DSHARED_PAGE_XR_TEST_BUILD=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

消费方在自己开启测试时把该选项打开：`bsp-guidance-action` 的 `User/tests` 与
`tools/guidance-contract` 都是这样的消费方，它们建 libxr、shared-page-xr 与本仓的测试
目标，然后 `ctest --test-dir /tmp/sg200x-app-tests` 运行契约测试。

`test_shared_page.cpp`：布局与发布索引——帧几何跟随注入 format、`TelemetryRing`/`Region`
跟随注入类型的偏移与总长、页头自描述互验与 `MISMATCH`（TAG/载荷尺寸/帧几何漂移）、
`Latest`/`Since` 的边界（SLOT_COUNT 槽余量、gap、新纪元）、region 稳定读与撕裂重试、
通道表（偏移由布局算出、通告与按名查找、越界与空名被拒）。
`test_coherency.cpp`：把记录型 provider 当作平台的答案，钉住意图→动作的映射——写路径不请求
invalidate、读路径不请求 clean、样本先于 `head` 发布、payload 先于 `seq` 发布、破 claim
先同步再推回、格式化是整页范围、通道表通告 EXCHANGE / 查找 READ。它不需要板子，因为
维护动作现在是注入的 provider。
`test_doorbell.cpp`：三个动词各走各的指针——`Wait` 原样传超时、`Signal` 不回执、
`Reply` 不响铃、拷贝后仍指向同一实现。
`test_command_channel.cpp`：通知侧是假的 `Doorbell`，逐条摆出“有通知/没通知/重复
通知”，钉住：无效载荷与未格式化页回执拒绝、重复通知只重发上次回执、`Ack` 之前不发
回执、空参数不消耗通知、通道自己从不 `Signal()`。

### 已知待办

`libxr` 尚未作为本仓库的 submodule pin 住（`.gitmodules` 里没有它），所以「单独构建
本仓库」目前要求旁边已有一份 libxr，或者从 `bsp-guidance-action` 的
`User/tests` / `tools/guidance-contract` 构建。补上后单独 `git clone
--recursive` 即可自带测试依赖。
