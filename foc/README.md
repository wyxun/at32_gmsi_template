# Modus FOC — 极简单电机矢量控制库

面向 STM32G431 单电机的极简 FOC 实现：**ops 表注入硬件端口 → 纯数学核心 →
MODUS Class 对象**。核心算法同时支持浮点（`float`）与 Q16.15 定点双后端。

架构只有三层：

```text
foc/app/foc_app.c        MODUS/product glue + foreground/ISR 调度
foc/motor/motor.c        单 Motor 生命周期、控制编排和速度分频
foc/hal/foc_port.h       ADC/PWM ops 表 — FOC 硬件边界
peripheral/stm32g431/foc_port.c   ops 默认实现（ADC 采样/校准、PWM、AS5600）
foc/middleware/foc_core.c         纯数学闭环（Clarke/Park/PI/IPark/SVPWM）
foc/observer/foc_encoder.c        AS5600 角度/速度观测（含 20 kHz 拍内外推）
```

## 1. 能力与边界

包含：

- 生命周期：`INITIALIZING → ADC_CAL → IDLE → ALIGN/RUNNING → FAULT`。
- 20 kHz 高频 ISR 数据流（ops 注入 + -flto 内联、无动态分配、无阻塞）：

```text
ADC ops current sample → PositionPort cached ReadFeedback()
→ foc_core_step → ptPwmOps->fnDutyCommit → (float 版) mwaveform.Step()
```

- 控制模式：`VOLTAGE`、`CURRENT`、`SPEED`、`POSITION`；其中 POSITION
  当前保留为明确返回 `FOC_RESULT_DISABLED` 的边界，速度 PI 在 20 kHz
  Motor 路径每 20 拍运行一次。
- 启动前 512 拍三相 ADC 零偏校准（PWM 关闭，~25.6 ms），偏移越界进 FAULT。
- 编码器角度拍内外推（低速保护阈值 1 e-turn/s），消除 1 kHz 采样阶梯。
- float 版波形直接读取 Motor/Core/PositionPort 的真实对象状态。
- Shell 命令：`motor start|spin|current|enc|cal|stop|clear|status`。

不包含（已从旧多实例框架收敛删除）：

- 多电机实例、开环状态、开环→闭环平滑过渡/资格判定/角度混合。
- 事件环、快照、Profile、旧 `foc_hal_*` 函数表、`motor_handle_t` 不透明句柄。
- 位置环、MTPA/弱磁/死区等高级算法（源码保留在仓库作参考，不参与构建）。

## 2. 目录

```text
foc/
├── math/          数值后端、BAM32 角度、三角（LUT/CORDIC/libm）
├── middleware/    foc_core.c/h — 纯数学闭环核心
├── control/       foc_pid.c/h — 电流/速度 PI（LADRC 等保留作参考）
├── modulation/    SVPWM
├── observer/      foc_encoder.c/h — 编码器观测与外推
├── motor/         motor.c/h — Motor 生命周期与控制编排
├── app/           foc_app.c/h — MODUS/product glue
├── hal/           foc_port.h — ADC/PWM 端口 API
├── doc/           内部架构文档
├── foc_config.h   编译期配置（数值后端、三角后端）
└── foc.h          极简伞头（含 Motor、PositionPort 和 App）
```

## 3. 数值后端与角度

- `FOC_NUMERIC_FLOAT` / `FOC_NUMERIC_FIXED` 必须且只能选一个
  （`foc_config.h` 编译期断言）；统一使用 `foc_scalar_t`。
- 角度使用 BAM32（`foc_angle_t`，uint32 满量程 = 1 圈），wrap 为零开销
  无符号溢出；单位统一为"圈"。
- 三角函数后端：`FOC_TRIG_BACKEND_LUT`（默认，host/无硬件加速芯片）、
  `FOC_TRIG_BACKEND_CORDIC`（STM32G431 硬件）、`LIBM`（精度对照）。

## 4. 对象模型与生命周期

`foc_app` 是符合 MODUS Class 规范的产品 glue 对象，拥有一个 `motor_t`：

- `motor_t` 以配置、控制、生命周期和调度四个内嵌运行态分组拥有 Core、
  速度 PI、PositionPort、请求、参考值、故障和反馈；App 不复制这些领域状态。
- `foc_app_cfg_t` 是初始化参数和外部依赖的唯一入口，直接包含 Motor 配置。
  Motor 配置持有电机物理参数、控制周期、编码器参数和 PositionPort。
- 实例由 `MODUS_DECLARE_OBJECT(foc_app, FocApp, ...)` 自动注册生成，
  唯一实例是 `tFocApp`；`foc_app.c` 中不再有隐藏的 `static foc_runtime_t`
  运行时。
- `foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)` 按模板
  契约校验并初始化传入的对象和配置；Shell 直接调用 Motor API。

生命周期状态：

| 状态 | 说明 |
|---|---|
| INITIALIZING | 初始化注册的 ADC/PWM/PositionPort |
| ADC_CAL | 累计零偏；超时/偏移非法 → FAULT；PWM 保持关闭 |
| IDLE | 功率级关闭；接受 start/align |
| ALIGN | 强制 CURRENT，固定 D 轴电流；完成后捕获零位并急停 |
| RUNNING | 高频链路运行；校准已完成，PWM 已使能 |
| FAULT | 第一动作 `foc_port_EmergencyStop()`；`motor clear` 在 PWM 关闭后复位 |

安全不变量（`tests/foc/test_foc_minimal_lifecycle.c` 覆盖）：

1. INITIALIZING/ADC_CAL/IDLE/FAULT 下 PWM 均关闭。
2. ADC_CAL 只读 ADC 累加零偏，不运行 FOC、不提交工作 duty。
3. RUNNING 前必须完成本次启动的零偏校准。
4. 进入 FAULT 第一动作是急停。
5. 角度无效 / ADC 失败 / 数学失败 / duty 提交失败 → FAULT。
6. STOP 无论软件状态如何先关 PWM。
7. PI 每次进入 RUNNING 前复位。
8. 中性 duty 先写入预装载寄存器，再使能 PWM 主输出。
9. 高频 ISR 中禁止日志、阻塞 I/O、动态内存和 I2C 访问。
10. AS5600 I2C 只在 foreground Poll 更新，ISR 只读一致性缓存。

## 5. 编码器路径与外推

- `foc_app_Run()` 通过 Motor foreground Poll 刷新样本缓存（I2C），Motor
  限制为最多 1 kHz，失败后退避 100 ms。
- 20 kHz ISR 中 `foc_encoder_Step()` 消费缓存：新样本到达时差分测速并
  低通滤波（α=0.25）；无新样本时按滤波速度外推
  `θ = θ_cached + ω × ticks_since_sample × T_hf`。
- `|机械速度 × 极对数| < 1 e-turn/s` 时不外推（低速量化噪声主导）。
- 磁铁失效或样本超时（默认 100 tick = 5 ms）→ `bValid=false` →
  RUNNING 中无有效角度进 FAULT。

## 6. 波形（float 版，真实对象状态直读）

`MWAVEFORM_MAX_CHANNELS=10`，`MWAVEFORM_SNAPSHOT_ENABLE=0`。注册于
`foc_app_Init`（`foc_app_WaveformInit`），ISR 每拍 `mwaveform.Step()`
直接采样对象成员内存（无 Push/快照/锁）：

| 通道 | 绑定变量（`tFocApp` 对象成员） |
|---|---|
| Iu / Iv / Iw | `tMotor.tControl.tCycleInput.qIu/qIv/qIw` |
| Id / Iq | `tMotor.tControl.tCore.tCurrent.qD/qQ` |
| Speed | `tMotor.tControl.tPositionFeedback.qElectricalSpeed` |
| Vd / Vq | `tMotor.tControl.tCore.tVoltage.qD/qQ` |
| Position | `tMotor.tControl.tPositionFeedback` 的缓存字段 |

fixed 版整个波形编译路径关闭（不注册、不 Step）。

## 7. 构建与测试

```powershell
# host 极简测试（core/encoder/lifecycle，float+fixed）
mingw32-make -C tests/foc clean
mingw32-make -C tests/foc minimal

# 固件（float / fixed；切换后端之间需 clean）
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel
mingw32-make clean
mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel FOC_NUMERIC=fixed
```

实测（2026-08-11）：float text≈54.3 KB / bss≈18.2 KB；fixed text≈53.7 KB /
bss≈18.2 KB；host 三套测试双后端全部 PASS。对象化重写后
（2026-08-31）：float/fixed 固件均可成功链接，host lifecycle 测试
覆盖对象初始化契约、初始化失败拒绝和多实例状态隔离。

## 8. Shell 命令

- `motor start` — VOLTAGE 模式启动（Vq=0.03），IDLE 才接受。
- `motor stop` — 立即急停并收敛软件状态。

AT32F413 当前没有注册 PositionPort，因此构建可通过但运行时 FOC 初始化
明确返回 `FOC_RESULT_DISABLED`，不会伪造开环反馈。
- `motor clear` — FAULT 且 PWM 关闭时清故障。
- `motor status` — 一次临界区复制输出：state/faults/pwm/angle/speed/
  Id/Iq/Vd/Vq/duty/calib。
- `motor current <id> <iq>` / `motor enc <speed>` — 设定对应模式参考并启动。

## 9. 参数整定顺序

1. 验证急停、硬件过流、PWM 极性。
2. 验证 ADC 触发点、三相零偏、电流方向。
3. 验证编码器电角度方向、极对数和零位。
4. 小 D/Q 电压验证开环相序与 Park 方向。
5. 开环角度下整定 Id/Iq 电流环。
6. 整定速度环。

详见 `foc/doc/foc-architecture.md`。
