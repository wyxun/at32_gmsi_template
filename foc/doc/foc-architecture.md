# Modus FOC 架构原理（极简单电机版）

本文档面向需要深入理解极简 FOC 库内部设计或进行二次开发的人员。使用接口
见 [`foc/README.md`](../README.md)。

## 1. 总体架构与模块分层

```
┌──────────────────────────────────────────────────┐
│  产品应用 / 中断 / Shell                          │
│  (target/stm32g4xx_it.c → foc_app_HighFrequencyISR) │
├──────────────────────────────────────────────────┤
│  foc_app.c   MODUS glue：foc_app_t tFocApp        │
│    ├── owns one motor_t                           │
│    └── Shell / waveform / scheduling              │
├──────────────────────────────────────────────────┤
│  motor.c    lifecycle + control orchestration     │
│    ├── ADC_CAL / ALIGN / RUNNING / FAULT          │
│    ├── cached PositionPort + foreground Poll      │
│    └── 20 kHz current loop + 20:1 speed loop      │
├──────────────────────────────────────────────────┤
│  middleware/foc_core.c   Clarke/Park/PI/IPark/SVPWM │
│  observer/foc_encoder.c  角度/速度观测 + 外推      │
│  control/foc_pid.c       电流/速度 PI              │
├──────────────────────────────────────────────────┤
│  math（foc_scalar_t、BAM32 角度、三角后端）        │
├──────────────────────────────────────────────────┤
│  foc_port.h   ops 表（pwm/adc/position）— 硬件边界  │
├──────────────────────────────────────────────────┤
│  peripheral/stm32g431/foc_port.c   ops 默认实现    │
│  (haladc / haltim1 / port_mdi / halcordic / as5600) │
└──────────────────────────────────────────────────┘
```

职责划分原则：

- **硬件只在 port/driver 访问**：采样、校准、PWM 提交、使能、急停全部
  收敛为 `foc_port.h` 与 `motor_position.h` 定义的 ops 表
  （`foc_pwm_ops_t`/`foc_adc_ops_t`/`motor_position_ops_t`）。foc 内部只通过注入的 ops 指针
  访问硬件，不直接接触 MDI/vendor。
- **ops 注入 + -flto 内联**：20 kHz 高频路径经过 ops 函数指针间接层，
  -flto 下内联为直接调用，成本趋零（详见 §10 设计决策）。多芯片/多
  传感器适配只更换 ops 表，foc 内部零改动。
- **算法层不依赖硬件**：`foc/math`、`foc/middleware`、`foc/observer`、
  `foc/app` 不包含任何厂商头文件、寄存器定义或芯片专用配置。
- **单实例**：`MODUS_DECLARE_OBJECT(foc_app, FocApp, ...)` 生成唯一
  `foc_app_t tFocApp`，App 状态全部属于该对象，不再有文件内静态
  runtime 与不透明句柄。

### 极简伞头

`foc/foc.h` 只导出公共算法与 App 入口。Motor 通过
`motor.h`/`motor_position.h` 提供单实例域对象，旧的 `foc_hal` 适配器不再构建。

## 2. 数值后端：浮点 / 定点双轨设计

### 2.1 类型系统

```c
#if defined(FOC_NUMERIC_FLOAT)
typedef float foc_scalar_t;     /* 硬件 FPU 或软浮点 */
#elif defined(FOC_NUMERIC_FIXED)
typedef int32_t foc_scalar_t;   /* Q16.15 定点 */
#endif
```

全部控制算法基于 `foc_scalar_t` 编写唯一一份代码；`foc_config.h` 编译期
断言必须且只能选一个后端。定点后端用 `foc_mul_pu()` / `foc_mul_wide()`
处理 Q16.15 乘法和饱和算术，关键路径不使用 `foc_from_float()` /
`foc_to_float()`。

### 2.2 主机测试双后端矩阵

`tests/foc/Makefile` 的 minimal-core / minimal-encoder / minimal-lifecycle
各以 float 与 fixed 各编译一次。任何算法改动必须通过双后端测试。

## 3. 角度模型：BAM32

```c
typedef struct {
    uint32_t wBam32; /* 0 到 2^32-1 表示 0.0 到接近 1.0 圈 */
} foc_angle_t;
```

- **零开销 wrap**：32-bit 无符号加法自然溢出即模 2^32 圈环绕。
- **角度差**：`target - actual` 无符号差按高位判方向，最短路径。
- **三角索引**：BAM32 高位直接用作 LUT 索引 / CORDIC 参数。
- 速度（turn/s）× 采样周期（s）在积分边界经 `foc_angle_from_scalar()`
  转 BAM32 后 `foc_angle_add()` 累加。

## 4. 三角后端三层架构

```c
#define FOC_TRIG_BACKEND_LIBM    0  /* 参考实现，仅用于精度对照 */
#define FOC_TRIG_BACKEND_LUT     1  /* 通用软件后端（host 默认） */
#define FOC_TRIG_BACKEND_CORDIC  2  /* STM32G4 硬件后端 */
```

- **LUT**：513 项 1/4 波表 + 线性插值，最大误差 < 1.2e-6，一次调用同时
  出 sin/cos。
- **CORDIC**：STM32G431 CORDIC 协处理器，~20–40 cycles；
  驱动在 `peripheral/stm32g431/halcordic.c`，FOC 内核零 vendor 依赖。
- `foc_angle_sincos()` 一次计算，`foc_park_cached()` / `foc_ipark_cached()`
  复用，避免重复三角调用。

## 5. 生命周期与请求边界

### 5.1 状态机

```text
INITIALIZING ──▶ ADC_CAL ──▶ IDLE ──▶ ALIGN ──▶ IDLE
       │             │          │        │
       └─────────────┴──────────┴────────┴──▶ FAULT
                                  IDLE ──▶ RUNNING
```

| 状态 | PWM | 说明 |
|---|---|---|
| INITIALIZING | 关 | 初始化注册的 ADC、PWM、PositionPort |
| ADC_CAL | 关 | ops 累计零偏；超时/失败 → FAULT |
| IDLE | 关 | 接受 start、recalibration 和 alignment 请求 |
| ALIGN | 开 | 固定 D 轴电流，完成后捕获 PositionPort 零位并急停 |
| RUNNING | 开 | 高频链路；进入前 PI 复位、中性 duty 先写再使能 PWM |
| FAULT | 关（急停） | 任何运行错误第一动作 `foc_port_EmergencyStop()` |

### 5.2 请求边界

Shell/产品代码直接调用 Motor API。Motor 只保留一个待处理动作；请求写入
使用一个短临界区，20 kHz Motor 路径消费动作。INITIALIZING 或 ADC_CAL
期间的启动请求直接返回 `FOC_RESULT_BUSY`，校准完成后仍保持 IDLE。
Stop 先急停再收敛软件状态，不使用通用命令邮箱或 BUSY 重试协议。

### 5.3 故障位

`tLifecycle.wFaults` 低 16 位：校准超时、校准失败、电流采样、角度无效、
数学失败、duty 提交、PWM 使能。

## 6. 20 kHz 高频数据流

```text
foc_app_HighFrequencyISR (ADC1_2_IRQHandler, 20 kHz)
 ├─ ADC_CAL: Motor calibration step (PWM off)
 ├─ ALIGN: fixed CURRENT reference, angle zero, duty commit
 ├─ RUNNING:
 │   ├─ ADC ops current sample → persistent motor.tCycleInput
 │   ├─ PositionPort cached ReadFeedback()
 │   └─ foc_core_step(&tCore, &tCommand, &motor.tCycleInput)
 │          Clarke → sincos → Park → Id/Iq PI(或电压参考) → IPark → SVPWM
 │      → PWM ops duty commit → TIM1 预装载寄存器
 └─ mwaveform.Step()  每拍采样真实 Motor/Core 变量
```

1 kHz `foc_app_Clock` 只做 MODUS 时间维护；PositionPort 的 I2C `Poll()`
只由 `foc_app_Run()` 调用，并由 Motor 做 1 ms 限频和失败后 100 ms 退避。
速度 PI 在 20 kHz Motor 路径每 20 拍运行一次。

### 编码器外推（消除 1 kHz 采样阶梯）

- 新样本到达：12 位码差分 / tick 数测速，低通滤波 α=0.25。
- 无新样本且未超时：`θ = θ_cached + ω_filtered × ticks × T_hf`。
- `|ω_mech × Pp| < 1 e-turn/s` 时不外推（低速量化噪声主导）。
- 样本超时（默认 100 tick = 5 ms）或磁铁失效 → `bValid=false` →
  RUNNING 无有效角度进 FAULT。

## 7. 安全不变量（`test_foc_minimal_lifecycle.c` 覆盖）

1. INITIALIZING/ADC_CAL/IDLE/FAULT 下 PWM 均关闭。
2. ADC_CAL 只读 ADC 累加零偏，不运行 FOC、不提交工作 duty。
3. RUNNING 前必须完成本次启动的零偏校准。
4. 进入 FAULT 第一动作是 `foc_port_EmergencyStop()`。
5. 角度无效、ADC 失败、数学失败、duty 提交失败 → FAULT。
6. STOP 无论软件状态如何都先关闭 PWM。
7. PI 每次进入 RUNNING 前复位。
8. 中性 duty 必须先写入预装载寄存器，再使能 PWM 主输出。
9. 高频 ISR 中禁止日志、阻塞 I/O、动态内存和 I2C 访问。
10. AS5600 I2C 只在 foreground Poll 更新，ISR 只读一致性缓存。

## 8. 波形（float 版）

`src/userconfig.h`：`MWAVEFORM_MAX_CHANNELS=10`、
`MWAVEFORM_SNAPSHOT_ENABLE=0`（快照静态缓冲随关闭移除，bss 降 ~1 KB）。
`foc_app_WaveformInit()` 注册真实 Motor/Core/PositionPort 成员；ISR 每拍
`mwaveform.Step()` 直接采样对象生命周期内的地址，无 App 镜像、浮点角度
转换或高频 `motor_GetFeedback()` 查询。fixed 版整个波形路径编译关闭。

## 9. 关键文件

| 文件 | 职责 |
|---|---|
| `foc/app/foc_app.c/h` | MODUS/product glue、调度、Shell、波形注册 |
| `foc/motor/motor.c/h` | 生命周期、请求/参考、20 kHz 控制、速度分频和反馈 |
| `foc/motor/motor_position.h` | 缓存 PositionPort 合约 |
| `foc/middleware/foc_core.c/h` | Clarke/Park/IPark + 电流环 PI 编排（纯数学，可 host 测试） |
| `foc/observer/foc_encoder.c/h` | 编码器样本消费、滤波测速、拍内外推 |
| `foc/hal/foc_port.h` | ops 表接口（pwm/adc/position）+ 默认实例声明，FOC 硬件边界 |
| `peripheral/stm32g431/foc_port.c` | ops 默认实现：ADC 采样/校准、TIM1 duty、急停、AS5600 编码器 |
| `foc/foc_types.h` | 公共值类型 + run_state/command/mode 枚举 |
| `tests/foc/test_foc_minimal_*.c` | core/encoder/lifecycle 双后端测试 |

## 10. 设计决策记录

### 为什么从多实例框架收敛为单实例对象

旧框架的多层 HAL、App 位置适配器和开环状态与当前编码器闭环职责重叠。
现在 Motor 通过四个内嵌运行态分组拥有 PositionPort、生命周期、参考和控制
运行态；App 只保留 MODUS/product glue。未来 V/F、I/F 以虚拟 PositionPort
接入，不向 Motor 继续添加开环字段。FOC_MODE_POSITION 当前明确返回
FOC_RESULT_DISABLED，待位置环实现后再开放。

AT32F413 没有注册 PositionPort，目标仍可编译，但 `foc_app_Init()` 明确返回
FOC_RESULT_DISABLED，避免在无位置反馈时伪造运行时 FOC。

### 为什么硬件接口用 ops 注入而非编译期函数

早期极简重构把硬件访问收敛为编译期 `foc_port_*()` 函数（无函数指针），
换来固定调用链；代价是每加一种外设/芯片/传感器都要改 `foc_port.h` 签名
或加条件编译，foc 内部无法与具体硬件解耦。ops 注入（策略模式）把
PWM/ADC/位置反馈抽象为三张表（`foc_pwm_ops_t`/`foc_adc_ops_t`/
`motor_position_ops_t`），foc 内部只面向接口：

- **加传感器**（如霍尔）：新增一个 `motor_position_ops_t` 实例注入槽位，
  foc 内部零改动；
- **换芯片**：更换 `foc_port.c` 提供的默认 ops 表，foc 内部零改动；
- **20 kHz 性能**：ops 函数指针在 -flto 下内联为直接调用，成本趋零
  （实测评估见 `docs/foc_app_HighFrequencyISR具体函数的资源占用优化方案
  (第三版).md`："函数指针改直绑 | 不做 | -flto 后成本趋零"）。

### 为什么编码器要外推

AS5600 1 kHz I2C 采样，20 kHz 控制若纯静态直读则 20 拍角度不变，50
e-turn/s 下阶梯滞后最高 18° 电角度。基于滤波速度的拍内外推把残差压到
<0.7°（速度量化 ±3.4% × 拍内 18°）；低速阈值防止量化噪声放大成抖动。

### 为什么波形直读变量

`mwaveform.AddVariable` 注册变量地址，`Step()` 每拍 volatile 直读，天然
单生产者（ISR）单消费者（RTT），无需快照拷贝与锁；bss 随快照关闭再降
~1 KB。

## 11. 延后扩展项

以下为有条件的扩展方向，当前不增加休眠代码：

- **SMO/NLFO 无感观测**：在 `foc_core` 旁挂并行观测器，读 αβ 电流/电压，
  输出候选角度；需先完成真机编码器闭环验证。
- **多电机实例**：需要把 `tFocApp` 实例化为数组并为每个实例配置独立
  port/中断，当前单实例约束下不做。
- **位置环 / 高级算法**：源码保留在仓库参考，确认需要后按安全门控接入。
