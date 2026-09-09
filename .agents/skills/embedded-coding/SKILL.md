---
name: embedded-coding
description: Use when 编写或审查 STM32/ARM/RISC-V 代码、MODUS Class、Mdriver、状态机、HAL/MDI 适配或 grblhal 集成时
---

# Embedded Coding Rules (modus / grblhal)

编写或审查嵌入式 C 代码时必须遵守本规则。规则按以下优先级执行：L0 MISRA 安全约束，L1 代码和对象设计，L2 状态机，L3 MODUS/MDI 库映射，最后是 TDD 验证。

本文档本身、其他 Markdown 文件、SKILL 文件、设计方案、测试报告和普通说明文字不受 82 字符硬限制。82 字符限制只适用于嵌入式 C/C++ 源代码；文档应按语义分段、表格完整性和阅读美观排版，不得为了凑行宽而切断自然段。

## L0: MISRA C:2012 安全子集（硬规则）

| 规则 | 要求 | bad | good |
|---|---|---|---|
| 10.3 | 禁止隐式整数转换 | `uint8_t b = a;` | `uint8_t b = (uint8_t)(a & 0xFF);` |
| 10.4 | 禁止有符号/无符号混合比较 | `if (s < u)` | `if (s < (int32_t)u)` |
| 13.1 | 禁止在 if 条件中赋值 | `if ((ret = f()) == OK)` | `ret = f(); if (ret == OK) {...}` |
| 13.2 | 禁止同一变量在表达式中多次修改 | `d = buf[i++]` | `d = buf[i]; i++;` |
| 13.5 | 复杂表达式中禁止自增/自减 | `d = buf[i++]` | `d = buf[i]; i++;` |
| 15.1 | if-else-if 链必须以 else 收尾 | 只有 if 和 else-if | 增加最终 else |
| 16.1 | switch 必须有 default | 没有 default | 增加 default 错误分支 |
| 16.3 | case 不能 fall-through | case 末尾没有 break | 每个 case 明确 break |
| 8.2 | 函数原型必须显式声明参数 | `void process();` | `void process(void);` |
| 8.13 | 只读指针必须使用 const | `void read(uint8_t *buf)` | `void read(const uint8_t *buf)` |
| 9.1 | 局部变量声明时必须初始化 | `uint32_t cnt;` | `uint32_t cnt = 0U;` |
| 17.7 | 函数返回值必须检查 | `hal_init();` | 保存返回值并处理错误 |
| 20.1 | include 必须位于文件开头 | 宏定义后 include | include 先于宏和代码 |
| 20.7 | 禁止使用 #undef | `#undef SIZE` | 使用作用域或独立命名 |

可选规则 15.5 默认不启用。只有项目规则明确打开时，才要求使用单一出口；嵌入式错误处理允许使用多个 return。

以下偏差允许保留：标准库内存操作 `memset/memcpy/memcmp/strlen`；DMA 缓冲区所需的 `void *` 指针运算；`third_party/` 中的 grblhal 内部代码。偏差不适用于 `src/`、`class/`、`peripheral/driver/` 和 `foc/` 业务代码。

## L1: 代码与对象设计

### L1.1: C 源码风格和复杂度

- 嵌入式 C/C++ 源码使用 82 字符硬限制。超出时使用反斜杠续行，续行缩进 4 个空格；优先在运算符或逗号后换行。此要求不适用于本文档或其他文档。
- C 源文件必须有 Doxygen 文件头，至少包含 `@file`、`@brief`、`@author` 和 `@date`。
- C 函数必须有 Doxygen 函数说明。至少包含 `@brief`、`@param` 和 `@return`；涉及 ISR、并发或错误边界时补充 `@note`。
- 行内注释只说明为什么存在硬件限制、性能考量、副作用或失败条件，不写“增加变量、循环、返回、赋值”等机械过程描述。
- 函数使用 `module_Action()` 命名，类型使用 `_t` 后缀，宏使用全大写。
- 函数体最多 80 行。统计函数定义大括号内的非空、非纯注释行；超过限制必须拆分职责。
- 一个函数只能有一个主要变化原因和一个主要动作。采样、数学计算、状态迁移、硬件提交、日志和参数解析应按职责拆分。
- 函数参数默认最多 4 个。超过 4 个时必须在评审中说明必要性，并优先将稳定配置放入配置结构体；不得为了逃避参数数量限制而随意打包结构体。
- 实时生产路径的实际调用深度最多 3 层；普通非实时生产路径最多 5 层。函数指针、回调、宏和 inline 展开后的调用层级也必须计算。
- 仅由明确的非 release 条件编译保护的调试路径最多 4 层；调试代码位于实时路径时仍最多 3 层。不能以“测试没有走到”为理由豁免。
- 复杂度超过限制时，优先拆成入口编排、业务步骤和叶子操作。不能只移动代码或增加宏来规避复杂度检查。

### L1.2: 结构体与操作 API

- 结构体必须保持单一职责。配置、运行状态、硬件接口、统计信息和错误上下文不能仅为了减少类型数量而合并。
- 直接成员数不超过 16 个为推荐范围；17～24 个必须在评审中说明成员的共同职责；超过 24 个默认拆分。寄存器映射、协议报文和厂商生成结构体可以例外，但必须标注用途。
- 具有生命周期、资源所有权、状态转换、不变量、硬件副作用、并发保护或中断访问约束的结构体属于对象，必须提供操作 API。API 负责维护不变量和错误返回。
- 纯数据结构不强制提供对象 API。坐标、采样值、abc/dq 量、命令和配置快照可以公开定义并按值或 const 指针传递；需要校验时只提供必要的 validate/normalize 接口。
- 禁止为了形式上的面向对象给每个成员机械增加 getter/setter。接口应表达完整操作或状态迁移，外部不得直接修改受保护对象的成员。
- `.c` 文件按模块职责和变化原因组织。只有在对象具有独立生命周期、独立测试入口或独立变化原因时，才拆成独立模块。
- 对于按 MODUS 模板构建的 Class，`xxx_t`、`xxx_cfg_t` 和直接组合的成员类型必须按模板在 `.h` 中提供完整声明，以便对象组合、配置组合和 `MODUS_DECLARE_OBJECT` 实例化显式可见。更深层的 Class 私有实现可以放在 `Mdriver/<class_name>/` 的私有头文件中。
- 结构体拆分不得破坏实时路径、增加无意义的调用层级或引入额外拷贝。FOC 和 ISR 数据还必须检查总大小、对齐、访问原子性、volatile 和缓存布局。

### L1.3: MODUS Class 与 Mdriver 组合规范（硬规则）

#### Class 的对象所有权

MODUS Class 的 `xxx_t` 是运行时状态的唯一拥有者。所有可变状态必须属于 `xxx_t`，或属于它直接拥有的子模块成员。

`xxx_cfg_t` 是初始化参数和外部依赖的唯一入口。配置字段必须有明确含义，并且必须在 `xxx_Init()` 中被校验、复制或绑定。禁止使用只有 `chReserved` 等占位字段的配置结构体掩盖没有配置契约的问题。

`xxx.h` 声明 `xxx_cfg_t`、`xxx_t` 和公开 API；`xxx.c` 实现行为并只放置私有辅助函数。Class 对象必须通过 `MODUS_DECLARE_OBJECT` 创建，禁止再在 `.c` 中创建一套替代对象的 `static` runtime。

如果 Class 是单实例，单实例必须是 `MODUS_DECLARE_OBJECT` 生成的 `xxx_t`，不能是“空的 MODUS 对象 + 隐藏的 `static xxx_runtime_t`”。Class 的结构体成员应按职责分组；复杂功能应组合成子结构体，不能把状态移到文件静态变量中。

#### 静态变量边界

Class `.c` 中允许的静态对象仅限于：

1. MODUS 模板要求的基础对象和基础配置，例如 `modus_base_t` 及其配置；
2. `const` 查表、只读映射和只读默认参数；
3. 明确声明所有权、生命周期和并发保护的共享资源管理对象；
4. 测试中的 fake、stub 和测试统计对象。

运行时状态禁止使用文件静态变量保存，包括状态、命令、定时器、缓存、PID 积分、编码器状态、错误标志、ISR 统计和 Waveform 句柄。若一个变量会随对象运行而变化，它必须是对象或对象子成员的一部分。

#### Mdriver 子模块

只有同时具备以下特征的 Class 子功能，才应抽离为 `Mdriver` 子模块：职责相对独立、功能较复杂、需要多个操作 API，并且未来具有较高的复用概率。此类子模块应有自己的结构体和 API，例如 `serial_screen_t` 与 `serial_screen_Run()`。

简单功能不应为了形式上的模块化而拆分，直接作为对应 Class 的成员和内部逻辑即可。

只服务于一个 Class 的复杂子模块放在：

```text
Mdriver/<class_name>/<submodule>/
```

预计会被多个 Class 独立实例化，且已经能够脱离父 Class 描述配置和依赖的通用子模块放在：

```text
Mdriver/<submodule>/
```

提升到 `Mdriver` 顶层必须同时满足：子模块较复杂且 API 较多、不依赖某个父 Class、API 不接收父 Class 指针、配置和依赖可以独立描述、每个父 Class 可以拥有独立实例，并且子模块可以独立测试。仅仅存在相似函数名，或仅仅为了预期中的复用，不足以抽成公共模块；简单子功能继续直接归属于对应 Class。

Class 专属的 Mdriver 是父对象的成员，不是独立 MODUS Class：

- 父 `xxx_t` 包含 `submodule_t` 成员；
- 父 `xxx_cfg_t` 包含 `submodule_cfg_t` 配置；
- 父 `xxx_Init()` 调用子模块初始化；
- 父 `xxx_Run()` 或父 PT 调用子模块服务 API；
- 子模块不能反向访问父 Class 内部成员；
- 子模块不使用 `MODUS_DECLARE_OBJECT`，除非它确实需要独立注册、独立生命周期或独立调度。

子模块接口应以子模块对象指针为上下文，例如 `serial_screen_Run(serial_screen_t *ptThis)`。子模块不得保存父 Class 指针，不得通过 `extern` 访问父 Class 的隐藏变量，也不得把父 Class 的业务决策塞进通用 Mdriver。父 Class 负责拥有、初始化、调度、停止和处理子模块错误。

多个 Class 共享一个物理资源时，必须使用显式 Manager 或接口表达所有权和仲裁关系。禁止使用隐藏的共享静态对象让多个 Class 偷偷共用资源。

#### Class 初始化契约

`xxx_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)` 必须按以下顺序执行：

1. 转换并检查对象地址和配置地址；
2. 校验配置内容；
3. 初始化对象自身状态；
4. 设置 `ptThis->ptBase` 和基础配置的 `wParent`；
5. 将配置依赖绑定到对象和子模块；
6. 初始化 PT 游标、业务状态、定时器、事件和子模块；
7. 通过 MDI 初始化硬件依赖，并检查每个返回值；
8. 最后调用 `mbase_Init()` 完成 MODUS 注册，并检查返回值。

启用 MODUS 的有效路径中不得用 `(void)wObjectAddr` 或 `(void)wObjectCfgAddr` 代替对象初始化。只有关闭 MODUS 的纯算法构建分支可以显式忽略这两个参数。

`Run` 和 `Clock` 回调必须从 `wObjectAddr` 获取真实的 `xxx_t`，并通过对象成员工作。除非回调确实无状态，否则禁止忽略对象地址。初始化失败时对象必须保持安全状态，已启用的硬件资源必须关闭。

#### Class 业务逻辑与 PT

- Class 的前台业务流程必须由 `Run()` 驱动，并使用对象成员保存的 perfc-PT 游标；业务状态和 PT 游标必须分开保存。
- 业务状态至少包含 `IDLE` 和 `ERROR`；状态转移只能在 Class 状态机内完成，与 ISR 共享的转移必须有中断保护。
- 等待、重试、通信、校准和多步骤流程必须使用非阻塞 PT 条件或 `perf_counter` 超时；禁止 `HAL_Delay`、忙等和阻塞式轮询。
- 每个等待状态必须有超时和错误收敛路径，默认超时为 500 ms。
- Class 专属 Mdriver 若拥有自己的流程，也必须在自己的对象成员中保存 PT 游标；父 PT 与子 PT 不得共享静态状态。
- `Clock()` 只执行短小的周期服务、设置事件或更新时间标志，不承载阻塞业务流程。Shell 只提交命令，不直接修改 Class 内部成员。

硬实时 ISR 是明确例外：FOC 电流环等固定时限路径可以使用确定性的直接调用链，但不能使用 PT、日志、I2C 或阻塞操作。ISR 所需的状态仍必须属于 Class 对象或其子模块成员，不能因此转移到隐藏静态 runtime。

#### Class 代码审查拒绝项

出现以下任一情况时，Agent 不得继续扩展实现，必须先回到对象设计：

- `xxx_t` 只有 `ptBase` 或占位字段，而真正状态在 `static` runtime 中；
- `xxx_cfg_t` 只有 `chReserved`，或存在未被初始化函数使用的配置字段；
- 可变状态、子模块、定时器或统计数据藏在 Class `.c` 的文件静态变量中；
- 启用 MODUS 时用 `(void)wObjectAddr` 或 `(void)wObjectCfgAddr` 跳过初始化；
- Class 的多步骤业务流程没有使用对象成员保存的 perfc-PT 状态；
- Class 专属子模块被错误注册成独立 MODUS 对象，或多个 Class 通过隐式全局变量共享资源；
- 修改 Class 行为时顺带修改 ADC、PWM、定时器或其他无关外设初始化，却没有单独的根因证据和验证计划。

## L2: 状态机策略（modus 真实能力）

库选择优先级：**perfc-PT 协程 > 裸机 switch > PLOOC**。

MODUS Class 和其 Mdriver 的前台业务流程优先使用 perfc-PT。裸机 `switch` 只适用于确实简单、无等待层次的叶状态机，不能用它绕开 Class 的 PT 约束。PT 游标必须存放在对应对象中，父对象和子对象各自拥有自己的游标。

### perfc-PT 协程（首选：任务级状态机/并发流程）

- 头文件：`perfc_task_pt.h`（Protothreads，开关 `__C_LANGUAGE_EXTENSIONS_PERFC_PT__`）和 `perf_counter.h`。
- 适用：多状态、需要等待、超时或挂起的流程型状态机。
- 示例：

```c
#include "perf_counter.h"
#include "perfc_task_pt.h"

typedef struct {
    struct pt pt;
    uint32_t timestamp;
    uint8_t count;
} pump_task_t;

static PT_THREAD(pump_run(struct pt *pt, pump_task_t *sm))
{
    PT_BEGIN(pt);
    for (;;) {
        PT_WAIT_UNTIL(pt, sm->count >= 10U);
        perfc_delay_ms(500U);
        sm->count = 0U;
        PT_YIELD(pt);
    }
    PT_END(pt);
}

void pump_init(pump_task_t *sm)
{
    PT_INIT(&sm->pt);
    sm->count = 0U;
}
```

### 裸机 switch（简单叶状态机）

裸机 `switch` 只用于无复杂等待层次的简单叶状态机。状态必须包含 `IDLE` 和 `ERROR`，每个可等待状态必须有超时和错误路径。

```c
typedef enum {
    SM_IDLE,
    SM_RUN,
    SM_ERROR
} sm_state_e;

typedef struct {
    sm_state_e eCurrent;
    sm_state_e eNext;
    uint32_t wTimeout;
} sm_t;

static void sm_Run(sm_t *ptThis)
{
    switch (ptThis->eCurrent) {
    case SM_IDLE:
        if (start_cond()) {
            ptThis->eNext = SM_RUN;
            ptThis->wTimeout = perfc_get() + 500U;
        }
        break;
    case SM_RUN:
        if (perfc_is_time_out_ms(500U, &ptThis->wTimeout, false)) {
            ptThis->eNext = SM_ERROR;
        }
        break;
    case SM_ERROR:
        break;
    default:
        ptThis->eNext = SM_ERROR;
        break;
    }
    ptThis->eCurrent = ptThis->eNext;
}
```

### PLOOC（复杂对象、继承和多实例）

- 头文件：`lib/plooc`，使用 `__IMPLEMENT`、`__new` 等宏。
- 适用：确实需要封装、继承和多实例的领域对象，例如电机或传感器管理器。
- Manager 结构体的第一个成员是基类，并使用 `__new` 创建实例。

### 通用状态机约束

1. 状态切换必须使用中断保护，或使用等效的调度保护。
2. 禁止在状态机中使用阻塞延时；协程使用 `perfc_delay_ms`，轮询状态机使用 `perfc_is_time_out_ms`。
3. 状态枚举必须包含 `IDLE` 和 `ERROR`。
4. 每个等待状态必须实现超时跳转，默认超时为 500 ms。
5. 状态机函数命名为 `xxx_Run()` 或项目现有的 `xxx_run()` 风格，初始化函数命名为 `xxx_Init()` 或对应项目既有风格；状态迁移只能发生在状态机内部。

## L3: MODUS 和硬件库映射（禁止重复造轮子）

| 场景 | 必须使用 | 禁止 |
|---|---|---|
| 对象注册 | `MODUS_DECLARE_OBJECT` | 手写注册表或自造对象框架 |
| Shell/日志 | `MODUS_SHELL_CMD` 和 mdebug | 裸 printf 自造日志协议 |
| 延时 | `perfc_delay_us`、`perfc_delay_ms` 或项目提供的等效宏 | 空循环延时、`HAL_Delay`、`vTaskDelay` |
| 超时判断 | `perfc_is_time_out_ms()`、`perfc_is_time_out_us()` | 手写毫秒计数比较 |
| 硬件访问 | MDI 层，即 `peripheral/<chip>/` 适配和 `peripheral/driver/` 芯片无关驱动 | 业务代码直接 include vendor HAL 或寄存器 |
| 调试 | `tools/aitrace.exe` 和 RTT | 裸串口自造 AI 调试协议 |
| 运动控制 | grblhal 的 `mc_`、`protocol_`、`settings_` API，经 `grblhal_adapt` 适配 | 业务代码直接调用 grblhal 内部实现 |
| Class 私有子模块 | `Mdriver/<class_name>/<submodule>/`，由父 Class 组合和调度 | 隐藏 `static` runtime 或父子反向耦合 |
| 多 Class 可复用子模块 | `Mdriver/<submodule>/`，每个 Class 持有独立成员实例 | 放入某个 Class 目录后跨 Class 直接引用 |
| 共享物理资源 | 显式 Manager、client 或接口注入 | 多个 Class 偷偷共用静态资源 |

### L3.1 日志级别选择（mdebug）

`MLOG/MLOGF` 级别不只是严重度，还承担**可观测性分类**。选择规则：

| 级别 | 使用场景 | 示例 |
|---|---|---|
| `T` | **周期性/重复性输出**（固定节拍、状态轮询、统计快照），运行期可单独开关（`log` 命令 bit4） | 每秒 ISR 计时报告、心跳、波形状态轮询 |
| `I` | **触发式一次性事件**（命令执行、状态迁移、校准完成） | `motor start accepted`、`encoder cal done` |
| `W` | 可恢复异常（命令被拒绝、依赖缺失、重试失败） | `motor start rejected (not IDLE)` |
| `E` | 不可恢复错误（初始化失败、硬件故障、断言） | `init failed`、`HardFault` |

核心约定：**周期性打印用 `T`，触发式用 `I`**。禁止把周期性输出写成 `I`（会淹没日志、无法单独关闭）；禁止把一次性事件写成 `T`（事件需要默认可见）。`W/E` 语义不变。高频 ISR 内禁止任何日志。

## TDD：独立验证要求

- 每个业务模块必须有独立验证入口：`tests/` 目录和可单独编译、运行的测试 target。
- 模块通过接口解耦，测试不依赖真实硬件；硬件依赖必须可注入 fake 或 stub。
- 修改模块行为时先补充或修改测试，再修改实现，形成红→绿→重构闭环；如果无法自动化，必须提供可执行验证脚本。
- 不为测试引入过度设计，测试接口应保持简单并服务于真实边界。
- 每个 MODUS Class 至少验证：空指针和配置校验、对象基类绑定、配置依赖注入、PT 流程推进、超时和错误收敛，以及初始化失败后的安全状态。
- 每个 Mdriver 至少有独立测试；父 Class 还必须有组合测试，验证每个子模块实例的状态互不污染。
- 如果多个 Class 使用同一个 Mdriver，测试必须覆盖两个独立实例；如果共享物理资源，测试必须覆盖 Manager 的所有权和仲裁接口。
- 代码审查前必须清点 Class `.c` 中所有 `static` 对象，并逐个归入允许的静态变量类别；无法归类的可变静态对象视为设计缺陷。

## Agent 执行流程

1. 读取 `AGENTS.md`，确认项目构建、调试、硬件和提交权限。
2. 完整阅读 `class/template_class.c` 和 `class/template_class.h`，再阅读目标 Class 的 `.h`、`.c` 及其测试。
3. 绘制对象所有权：标出 `xxx_t`、`xxx_cfg_t`、每个 Mdriver 成员和每个外部依赖；列出 `.c` 中所有 `static` 变量及其允许性。
4. 判断子功能是简单 Class 成员、Class 私有 Mdriver、可复用 Mdriver，还是必须独立注册的 MODUS Class。
5. 按任务选择状态机档位和库映射；Class 前台流程默认使用 perfc-PT，硬实时 ISR 按明确例外处理。
6. 修改行为前先建立最小失败测试或可执行复现。
7. 生成代码后逐项自查 MISRA、源码 82 字符、Doxygen、返回值、函数体长度、调用深度、结构体职责、对象 API、Class/Mdriver 静态状态和 PT 所有权。
8. release 构建必须验证调试代码已被预处理移除；debug 构建允许的最大调用深度为 4 层。
9. 运行 Class 和 Mdriver 的独立测试、目标构建和与风险匹配的硬件验证；没有硬件证据时不得宣称真机行为已验证。

## Agent 自检红线

如果 Agent 发现以下任一情况，必须停止继续堆叠修改并回到对象设计：

- 为了快速通过编译，把运行状态放入 `static` 变量；
- 用 `chReserved` 保留没有实际用途的配置结构体；
- 只初始化 `ptBase` 和 `mbase_Init()`，但没有初始化对象成员、配置依赖和子模块；
- 因为单实例而创建隐藏 runtime，导致 MODUS 对象成为空壳；
- 用一个父对象或全局变量保存多个子模块的 PT 进度；
- 把简单功能过度拆成 Mdriver，或把复杂且可能复用的功能散落在 Class `.c`；
- 让多个 Class 通过隐式全局对象共享物理资源；
- 为修复 Class 业务问题而无证据修改 ADC、PWM、定时器或其他无关外设初始化；
- 把文档的自然排版误认为 C 源代码的 82 字符规则。
