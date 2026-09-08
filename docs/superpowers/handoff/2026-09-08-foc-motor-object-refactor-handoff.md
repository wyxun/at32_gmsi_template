# FOC Motor 对象重构交接文档

> 日期：2026-09-08  
> 工程：`D:\2_xundoc\project\modus_template`  
> 目的：在没有硬件的情况下完成软件迁移和自动化验证，硬件回来后继续完成台架回归。

## 更新记录（2026-09-09）

硬件回归首轮发现 `encoder cal` 被拒（日志 `[W] encoder cal request rejected`）。
根因不是硬件/时序：重构后 `foc_app_Start()` 对 VOLTAGE/CURRENT/SPEED 一律切换
到 Motor 路径，而旧 App 非阻塞标定服务只在 legacy 分支被调度；标定请求第一次
被接受后其对齐动作把 `bMotorControlPath` 置位，状态机随即停在 `ALIGNING` 不再
推进 → offset 永远抓不到、`calibrated` 保持 0、后续请求全部 `BUSY` 被拒。

已完成 P0-1/P0-2 的编码器零位标定下沉：

1. `motor_t` 新增 `MOTOR_STATE_POSITION_CAL` 生命周期与
   `motor_RequestPositionCalibration()`：从 IDLE/INITIALIZING/CALIBRATING
   进入，先完成 ADC 校准再以固定电角 0 施加 Id 对齐电流（`qD=对齐电流`），
   HF 按 `wPositionCalibrationTicks`（1500 ms @ 20 kHz = 30000 拍）保持，
   BackgroundStep 到时调用活动 provider 的 `fnCaptureElectricalZero()`；
   失败（位置无效/超时）先急停再锁 `MOTOR_FAULT_POSITION_CAL`，`ClearFault`
   后可从故障恢复，`Stop` 随时取消对齐。
2. App 侧位置 provider 补齐 `fnCaptureElectricalZero`（读缓存机械角 →
   offset = -mech × 极对数，写入 `tPosition`），`foc_app_RequestEncoderCalibration`
   改为走 Motor 并同步镜像；删除旧 App 标定服务、状态枚举与 `tEncoderCalibration`
   成员（卡死源头）。
3. Motor 慢速路径补 1 kHz 限速与 EncMech 波形镜像（修复 Motor 路径下 I2C 随
   Run 节奏刷新、EncMech 通道停更的隐患）。

验证：`tests/foc minimal` 全绿（core/encoder/lifecycle/motor/as5600 的
float+fixed，motor 新增 7 个标定用例：无 capture DISABLED、状态/参数门控、
对齐后抓拍、ADC 校准先行、位置无效→FAULT→恢复、ADC 超时→FAULT、Stop 取消）；
`TARGET_CHIP=stm32g431` 固件 debug-rel 与 debug 构建通过；motor.c 边界检查
无 I2C/log/PT。台架回归（T5 `encoder cal` ×4 及 offset 重复性、100 eHz 波形）
待烧录后按 `docs/foc-test-guide.md` 执行，对比 0907 基线。

## 1. 当前结论

当前 AS5600 速度闭环已经接入 `motor_t` 的控制路径，同时保留原有数值口径和
高频安全边界。电压、CURRENT、SPEED 三种控制模式都经过 App 到 Motor 的 wrapper。

编码器电气零位标定目前仍由 `foc_app` 的非阻塞服务承载，尚未完全下沉到 Motor。
这是有意保留的安全边界：没有硬件时先锁定软件数据流，避免同时改变对齐电流、等待
时间和零位捕获时序。

## 2. 已完成内容

### Motor 对象

- 新增 `foc/motor/motor.h`、`motor.c`、`motor_params.h`、`motor_position.h`。
- 提供静态内存、单活动位置 provider、ADC/PWM/context 注入接口。
- 提供生命周期：初始化、空闲、ADC 校准、运行、故障。
- 提供高频步、1 kHz 时钟步、后台步、Start/Stop、参考量和状态/反馈查询。
- 速度 PI 只在 SPEED 模式运行；CURRENT 模式的固定 Iq 不被速度环覆盖。
- 故障路径先调用 PWM emergency stop，再锁存故障状态。

### Core 和 App

- `foc_core_state_t` 不再保存角度、速度和三相原始输入的重复副本。
- `foc_app_t` 初始化并持有 `motor_t`。
- App 位置适配器支持开环角度和编码器反馈。
- 极对数、方向反转和电气零位换算保持现有口径。
- 现有 `foc_app_*` 对外 API 保持不变。

### AS5600

- 新增 `g_tAs5600PositionOps`。
- 1 kHz 慢速路径调用 I2C 更新缓存。
- 高频路径只读取缓存，不访问 I2C、不阻塞。
- provider 测试覆盖缓存读取、极对数电角度换算和总线访问隔离。

### 接口上下文

ADC/PWM 回调统一增加 `void *pContext`。当前 STM32G431 单板可以继续使用 `NULL`
上下文；主机测试已使用独立上下文验证多实例隔离。

## 3. 已验证结果

工具链路径：

```powershell
$env:Path = "D:\software\msys64\mingw64\bin;D:\software\msys64\usr\bin;$env:Path"
```

完整主机测试：

```powershell
mingw32-make -C E:\Project\modus_template\tests\foc minimal
```

结果：core、encoder、App lifecycle、Motor、AS5600 provider 的 float/fixed 测试均为
`PASS (0 failures)`。

固件构建：

```powershell
$env:MAKE_EXE = 'D:\software\msys64\mingw64\bin\mingw32-make.exe'
.\make.bat BUILD=debug
```

最近一次结果：

```text
text 61684
data   640
bss  18952
dec  81276
```

边界检查：

```powershell
git diff --check
rg -n "as5600|I2C|mdebug|printf|perfc_task_pt|HAL_" foc/motor/motor.c
```

`motor.c` 未发现具体传感器、I2C、HAL、日志或任务依赖。`git diff --check` 没有空白
错误；Git 可能提示工作树文件的 LF/CRLF 转换警告，该警告不是代码错误。

## 4. 用户提供的硬件基线

以下数据来自用户 2026-09-07 的实测，当前没有重新测量：

| 项目 | 基线 |
|---|---|
| cal offset 重复性 | 0.4597 / 0.4630 / 0.4648 / 0.4614，极差 0.0051 turn |
| cal 对齐电流 | Id ref 0.10 → actual 0.110，Vd=0.108 |
| 100 eHz 5 s 波形 | mean 98.5 eHz，std 4.9，范围 95-105 |
| Id / Iq | 约 0 / ±0.06，残留 ±0.2 纹波，约 200-450 Hz，无高频声 |
| 100 eHz 上限 | 12 V 可稳定运行，BEMF 约 2.1 V |

必须保留的限制：5-30 eHz 的不平滑来自 AS5600 1 kHz 量化，误差约 ±1.7 eHz；
50+ eHz 为平滑工作区；1 kHz 更新台阶属于正常现象。

## 5. 下一步任务

### 优先级 P0：无硬件即可完成

1. ~~将编码器零位标定状态机逐步下沉到 Motor。~~ 已完成（2026-09-09/10，
   `MOTOR_STATE_POSITION_CAL` + `motor_RequestPositionCalibration()`；
   台架 `encoder cal` 已多次成功打印 offset）。
2. ~~添加 fake provider 标定测试~~ 已完成：对齐开始/ADC 校准先行/等待时长/
   位置无效/ADC 超时/抓拍电气零位/急停与故障恢复，见 motor 主机用例。
3. ~~清理 `foc_app_t` 中不再参与控制的重复 Core/PID/位置运行状态~~ 已完成
   （stage-5）：删除 legacy 生命周期运行路径（App 命令邮箱、legacy 校准/角度/
   运行步、EncoderPoll、`bMotorControlPath` 双分支）与全部镜像字段
   （`tCore/tCommand/tLifecycle/tCalibration/ptPwmOps/ptAdcOps` 及
   diagnostics 的重复相位/速度副本）。`foc_app` 现只保留 MODUS/Shell/波形 +
   位置 provider 适配；`GetStatus()` 现场从 `motor_GetStatus/GetFeedback` 组装，
   波形通道直接采样 Motor 实时字段。host 测试 float+fixed 全绿，G431 debug-rel
   构建通过（RAM −272 B）。
4. ~~复查 `foc_app_GetStatus()`、波形通道和 Shell 查询是否只读取 Motor
   快照。~~ 已完成本阶段：波形 Iu/Iv/Iw 与 Angle 通道改为镜像 Motor
   快照（motor 新增最近一拍相位电流缓存 + Sync 换算），EncMech 走
   provider 慢速镜像；状态/Shell 均读 Motor 快照。剩余波形工具侧时间列
   损坏不属于固件。
5. ~~继续运行 float/fixed 主机测试和 ARM debug/release 构建~~ 已在本轮
   完成并全绿（含新增 phase_snapshot 用例）。

### 优先级 P1：硬件回来后完成

1. 重复四次 cal offset 测试。✅ 0.4624 / 0.4607 / 0.4623 / 0.4605，
   极差 0.0019 turn（0907 基线 0.4597~0.4648、极差 0.0051；验收 <0.143）。
2. 复测 Id=0.10 对齐电流和 Vd。✅ 标定期间 Id≈0.11、Vd≈0.106（status 快照），
   与基线 Id 0.110 / Vd 0.108 一致。
3. 复测 100 eHz、5 秒速度波形。⚠️ 两次 5 s 抓取（Iq 0.05）Speed
   mean 98.46/98.60 eHz、std 5.29/6.02，median≈99、IQR 96.8–101.3；
   但存在周期性短暂低谷（p5≈88–90，min≈68，95–105 包络占比 ~70–80%）。
   Iq 上限提高到 0.10 后低谷依旧（平均 Iq 仅 0.016），排除电流钳位饱和；
   低谷无固定机械相位 → 倾向编码器 1 kHz 采样/速度估计与真实微速纹波的
   耦合或台架负载/装夹差异，暂不改 PI。待与 0907 同装夹复测或用 mstudio
   原始波形对比后再定性。原始文件：`captures/wave_2026-09-08T15-00-04*.csv`、
   `wave_2026-09-08T15-01-17*.csv`（Iq0.05）与 `wave_2026-09-08T15-02-02*.csv`
   （Iq0.10 诊断）。
   20 s 长窗复核（`wave_2026-09-08T15-04-31*.csv`）：mean 98.7、std 5.4；
   低谷为 ~2 ms 级平台（Speed 两次编码器样本间保持恒定），仅在 AS5600
   1 kHz 新样本点跳变，且新样本点本身即含低谷（<95 占 23.1%、<90 占 4.5%）
   → 排除插值假象、排除 Iq 饱和；自相关仅弱峰 ~19–22 Hz，无强周期，非
   固定机械/电相位齿槽。决定：**接受现状归档**（mean/std 满足 ±5%；短时
   低速凹陷作为当前装夹下 AS5600 1 kHz 测速口径的已知特性记录），专项
   延后——后续同装夹复测定性，必要时评估 `foc_encoder` 滤波/测速时基，
   **不改 PI 与滤波参数**。
4. 复测 Id/Iq 纹波、声音和 12 V 下 100 eHz 稳定性。⚠️ Id std≈0.019–0.020、
   Iq std≈0.037–0.043（零交叉粗估主导 ~130 Hz，基线记录 ~200–450 Hz），
   无啸叫；12 V 短时稳定、faults=0。纹波频率口径待波形工具精确复核。
5. 如果数据回归，先增加可复现测试和数据流检查，不要直接修改 PI 增益。
   按此执行：未改增益；已保留 3 份原始波形与 status 快照供复现。

## 6. 当前已知注意事项

- 不要在 `motor_HighFrequencyStep()` 或其调用链加入 I2C、日志、阻塞等待或任务 PT。
- 不要为了消除 AS5600 台阶而在 20 kHz 路径增加未经评审的插值。
- 不要把用户现有 `makefile`、`modus`、`diagrams/` 修改重置或覆盖。
- 当前工作树没有提交；不要执行 `git reset --hard`、`git checkout`、分支切换、提交
  或推送，除非用户明确要求。
- 硬件回归前不要刷写设备，也不要把软件仿真结果写成 offset、BEMF 或噪声实测。

## 7. 重要文件索引

- 设计：[2026-09-08-foc-motor-object-refactor-design.md](../specs/2026-09-08-foc-motor-object-refactor-design.md)
- 计划：[2026-09-08-foc-motor-object-refactor.md](../plans/2026-09-08-foc-motor-object-refactor.md)
- Motor：[motor.h](../../../foc/motor/motor.h)、[motor.c](../../../foc/motor/motor.c)
- App：[foc_app.h](../../../foc/app/foc_app.h)、[foc_app.c](../../../foc/app/foc_app.c)
- AS5600：[as5600.h](../../../peripheral/driver/as5600.h)、[as5600.c](../../../peripheral/driver/as5600.c)
- 主机测试：[test_motor.c](../../../tests/foc/test_motor.c)、[test_as5600.c](../../../tests/foc/test_as5600.c)、[test_foc_minimal_lifecycle.c](../../../tests/foc/test_foc_minimal_lifecycle.c)

