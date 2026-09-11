# FOC 当前代码审查报告

审查日期：2026-09-11  
审查范围：`foc/`、`peripheral/stm32g431/`、
`peripheral/at32f413/` 当前工作树  
审查方式：静态调用链、状态机、配置所有权、并发访问和目标构建检查。  
本轮只审查，不修改源码。

## 结论

当前代码已经形成了较清晰的主链路：

```text
AS5600/I2C -> Encoder(1 kHz 前台缓存) -> Motor(20 kHz ISR)
ADC/PWM    -> foc_port               -> Motor -> FOC Core
```

STM32G431 的 debug 构建可以完成，现有 ADC U/V/W 映射、采样极性、PWM
桥控制和 AS5600 初始化/读取没有发现被这次重构删除的问题。

但是，当前还不能称为“可冻结基线”。有两个会直接影响运行安全的缺陷，
一个会使故障退避失效；同时仍有若干配置死字段、冗余回调和未清理的旧代码，
会继续推动源码膨胀。

## 必须先修复的问题

### P0：ALIGN 完成时电气零位符号/数据源错误

位置：`foc/motor/motor.c:272-285`

当前流程先把机械位置转换成“已经减过旧零位”的电气角，随后再取反保存
为新零位。初始零位为零时，保存结果为 `-base`，下一拍运行角度变成
`base - (-base) = 2 * base`。这不是简单的偏差，而是闭环换相角错误，可能
造成抖动、反向或失控。

ALIGN 完成时应直接用当前机械 BAM32 乘以 Motor 唯一持有的极对数，截断到
32 位自然完成模 `2^32`，不能复用已经应用电气零位的运行输入转换函数。

### P1：`motor_Stop()` 可以永久打断 ADC 校准

位置：`foc/motor/motor.c:359-373`

`motor_Stop()` 在 `MOTOR_STATE_ADC_CAL` 状态也无条件改成 `IDLE`，但不会把
`bIsCalibrated` 设为真，也没有重新校准 API。此后 `motor_Start()` 会因为
“未校准”持续返回 `FOC_RESULT_BUSY`，对象无法自行恢复，只能重新初始化。

需要明确状态契约：校准期间 Stop 要么拒绝，要么重新进入 ADC_CAL；不能把
“未校准的 IDLE”作为可恢复状态暴露出去。

### P1：ADC 校准故障清除后同样无法恢复

位置：`foc/motor/motor.c:375-390`

ADC 校准失败进入 FAULT 后，`motor_ClearFault()` 只清故障并进入 IDLE，
没有调用 `foc_adc_CalibBegin()`，而 `bIsCalibrated` 仍为 false。因此清障后
仍不能 Start。应规定清除 ADC_CAL 故障时重新启动校准，或明确要求重新 Init；
不能让 API 表面返回 OK 但对象实际不可启动。

### P1：Encoder 读失败没有启动 100 ms 退避

位置：`foc/app/foc_app.c:181-191`

失败分支和成功分支都把 `lBackoffTimestamp` 清零，导致每个 1 ms 前台周期
都会再次调用 I2C 读取。这样既违背设计中的退避意图，也会在总线异常时放大
CPU 和 I2C 负载。

失败时应记录当前 tick；下一次只有超时后才重试，成功后再清零。

### P1：Shell 命令可能“启动成功、设置参考失败”，留下带 PWM 的电机

位置：`foc/app/foc_app.c:251-283`

`speed/current/voltage` 命令先调用 `motor_Start()`，再调用对应的
`motor_Set*Reference()`。如果第二步因状态、模式或参数失败，Motor 已经进入
RUNNING，PWM 仍然保持使能，参考值可能是零或上一次默认值。

应使用原子化的“带初值启动”接口，或在设置参考失败时立即 `motor_Stop()`。
这是控制入口的安全事务问题，不应由 Shell 调用顺序隐式承担。

## 需要在冻结前收口的问题

### P2：`wForegroundPeriodUs` 是无效配置

位置：`foc/app/foc_app.h:18`、`foc/app/foc_app.c:176`

配置提供了 `wForegroundPeriodUs`，初始化时也填了 1000，但运行代码使用硬编码
的 `1000U`，没有把配置保存到对象。修改配置不会改变行为，形成配置与实现的
双事实来源。应删除字段，或保存并使用它；按 KISS 原则更建议只保留一个来源。

### P2：Encoder 的 `qHighFrequencyPeriod` 当前只校验、不参与计算

位置：`foc/observer/foc_encoder.h:23`、`foc/observer/foc_encoder.c:173`

该字段在配置中占用空间，在板级初始化中还被赋值，但 Encoder 的速度计算、
外推和超时计算都没有读取它。它目前不是功能参数，只是死配置。应删除，或
明确它属于 Motor/调度器并在唯一位置使用；不能继续保留“看似有用”的字段。

### P2：物理参数与当前电流环实现脱节

位置：`foc/motor/motor_params.c:12-24`、`foc/motor/motor.c:306-315`

`chPolePairs` 参与电角度和速度换算；电阻、Ld、Lq 目前只做非零校验并复制
到 `motor_t`，电流 PI 仍完全使用手工填写的 `tCurrentPiParams`。如果设计声称
这些参数用于电流环初始化，则文档与代码不一致；如果本轮暂不使用，则应在文档
中标注为保留元数据，避免误以为已经生效。

### P2：速度 PI 重启时没有复位

位置：`foc/motor/motor.c:189-203`、`foc/motor/motor.c:291-323`

`motor_Start()` 和 `motor_Stop()` 只重置 FOC Core 的电流 PI，不重置
`tSpeedPi`。上一次 SPEED 运行留下的积分项会带入下一次启动，可能产生瞬时
Iq。进入 RUNNING 前应复位速度 PI，或在状态机中明确定义保留积分的语义。

### P2：位置捕获回调是纯转发层

位置：`foc/motor/motor.h:37-45`、`foc/app/foc_app.c:45-103`、
`foc/observer/foc_encoder.c:281-285`

`fnCapturePositionZero -> foc_app_CapturePosition ->
foc_encoder_CaptureZero -> foc_encoder_GetPosition` 最终只是同一次机械位置
读取。当前 Encoder 已不维护电气零位，ALIGN 只需使用同一个机械位置读取接口。
这条链增加了配置字段、函数指针和绑定代码，却没有增加语义。建议删除捕获
回调，ALIGN 直接使用 `fnGetPosition`。

### P2：双缓冲发布缺少明确的 ISR/前台可见性契约

位置：`foc/observer/foc_encoder.c:144-153`、`253-270`

双缓冲的“先写非活动槽、最后切索引”逻辑在单核硬件上方向正确，但
`chPublishedIndex`、`bHasSample` 和槽内容都是普通 C 对象，前台与 ISR 异步
访问，代码没有声明可见性/编译器重排约束。应至少把发布索引和有效标志的并发
契约写清并验证编译器输出；若采用 `volatile` 或现有中断保护，也应只保护
发布点，不要恢复序列号自旋等待。

### P2：硬件停止在 FAULT 高频路径重复执行

位置：`foc/motor/motor.c:21-29`、`501-524`

进入故障时已经调用一次 `foc_pwm_Stop()`，之后每个 20 kHz 周期在 FAULT
状态再次调用。若底层 MDI/定时器停机不是严格常数时间，这会浪费 ISR 预算；
即使是常数时间，也使状态动作和状态保持混在一起。建议只在故障状态边沿停机，
或明确底层 Stop 必须幂等且可接受每拍调用。

### P2：硬件错误被统一压成 `INVALID_ARGUMENT`

位置：`foc/observer/foc_encoder.c:193-198`、`219-223`

传感器初始化失败和 I2C 读取失败都映射为 `FOC_RESULT_INVALID_ARGUMENT`。
上层无法区分参数错误、硬件不可用和运行时通信故障，因而不能做准确的退避、
诊断或故障统计。至少应在 Encoder 内部保留失败类别，或增加明确的硬件/通信
错误结果码。

### P3：模式判断有冗余条件，旧枚举仍留在公共头文件

位置：`foc/motor/motor.c:334`、`foc/foc_types.h:46-58`

`eMode >= FOC_MODE_POSITION` 已经覆盖 `eMode >= FOC_MODE_MAX`，后者是重复
判断。`foc_run_state_e` 和 `foc_command_e` 在当前构建链中没有使用。它们会让
公共 API 看起来仍支持另一套状态/命令模型，应删除或移到明确的实验目录。

### P3：Shell 参数解析接受前缀垃圾字符

位置：`foc/app/foc_app.c:243-284`

`strncmp(args, "stop", 4)` 等判断会接受 `stopxyz`；`sscanf` 也允许数值后
附带未检查文本。对调试 Shell 不是实时缺陷，但会造成命令行为不透明。应检查
命令词后的空白/字符串结束，并验证完整参数串。

## 源码持续增加的根因

当前 `foc/foc.mk` 只把核心数学、PID、SVPWM、Encoder、Motor 和 App 纳入构建，
但仓库仍保留 HFI、SMO、NLFO、开环、实验和多种高级控制模块，并把这些目录
全部加入 include 路径。这样会产生三类认知负担：

1. 读者无法快速区分“当前框架接口”和“未来算法参考实现”；
2. 旧接口容易被误重新接入，继续形成两套抽象；
3. 头文件中的遗留类型与当前 Motor 状态机并存，削弱 SSOT。

如果这些模块当前确实只是参考，应移到明确的 archive/experimental 边界，或
从公共 include 路径移除；如果不是本轮范围，不要继续为它们增加适配层。

## 已核对且建议保持不动的硬件约束

- STM32G431 的 ADC 映射仍为 U=`ADC1 injected[0]`、
  V=`ADC2 injected[1]`、W=`ADC2 injected[0]`。
- STM32 电流换算仍保持 `offset - raw`；AT32 保持其现有
  `raw - offset` 板级极性，不能为了统一代码形式而改相序/符号。
- PWM 仍经 TIM1/MDI（STM32）或现有 `halpwm`（AT32）输出，停机路径保留。
- STM32 的 ADC 触发启动和 AS5600 `Init`/I2C 原始角度读取仍在
  `peripheral/stm32g431/foc_port.c`。

## 建议处理顺序

1. 先修 P0/P1：ALIGN 零位、校准状态恢复、Encoder 退避、启动参考事务。
2. 再做 P2 收口：删除死配置和捕获回调，明确双缓冲可见性，复位速度 PI。
3. 最后做 P3/仓库减法：清理旧枚举、严格 Shell 解析，并隔离未构建算法目录。
4. 完成后再做 float/fixed、Encoder 调优和无感 shadow；不要在上述状态机问题
   未闭合前叠加新的无感接管逻辑。

## 构建证据与验证缺口

本轮执行 `mingw32-make BUILD=debug`，目标构建返回成功，输出 ELF 大小为
`text=53584`、`data=688`、`bss=18560`。当前工作树未见可用的原有
`tests/foc` 测试目标，因此构建成功不能替代 ALIGN、校准故障恢复、I2C 退避和
启动失败回滚的行为测试。新增或恢复测试需要按项目权限另行确认，不应在本报告
中默认为已验证。
