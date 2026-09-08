# FOC Motor 对象重构——硬件回归与收尾测试报告

> 日期：2026-09-10
> 环境：STM32G431 + AS5600 + 2205 电机（7 对极）+ 12 V 母线 + CMSIS-DAP/RTT
> 范围：09-08 重构后的台架回归、三个回归问题定位与修复、foc_app stage-5 收尾，
>      及 float/fixed 主机测试与 G431 固件构建验证。
> 基线：2026-09-07 实测（docs/foc-test-guide.md §3a、design §3a）

## 1. 结论摘要

| 项目 | 结果 |
|---|---|
| `encoder cal` 重复性（×4） | offset 0.4624/0.4607/0.4623/0.4605，极差 **0.0019 turn**（基线 0.0051，验收 <0.143） |
| 100 eHz 速度闭环 | RUNNING、speed 采样 103.2/94.5 eHz；波形 mean 98.3–98.7、std 5.2–6.0（±5% 内）；短时 1 kHz 级凹陷按“已知特性”归档 |
| 停止/重启/反转 | `motor stop` 立即 IDLE；停后再次启动、`motor enc 0.10 -50`（speed −48.0）均正常 |
| 全程 faults | 0 |
| 主机测试 | `tests/foc minimal` float+fixed 全 PASS（core/encoder/lifecycle/motor/as5600） |
| 固件 | G431 clean debug-rel 构建通过（text 59940 / data 640 / bss 18648） |

本次发现并修复的回归全部为**软件数据流/生命周期**问题，非硬件或控制增益问题；
按计划“先查数据流、不改 PI”执行，未修改任何 PI/滤波参数。

## 2. 台架实测数据

### 2.1 编码器零位标定（T5 前置）

| 次数 | offset（turn） | 等效抓拍机械角（turn） |
|---|---|---|
| 1 | 0.4624 | ≈0.9360 |
| 2 | 0.4607 | ≈0.9341 |
| 3 | 0.4623 | ≈0.9359 |
| 4 | 0.4605 | ≈0.9336 |

- 极差 **0.0019 turn** < 基线 0.0051 < 验收 0.143，数值落在 0907 基线族
  （0.4597~0.4648）内。
- 标定期间 status：Id≈0.11、Vd≈0.106（与基线 Id 0.110/Vd 0.108 一致，对齐
  电流 0.10 pu 电流环跟踪正常）。
- 注：早期一次抓到的 0.4482/0.4448 为一次性偏差，未在本次四连测中复现。

### 2.2 100 eHz 速度闭环（`motor enc 0.05 100`）

status 采样：speed 103.2 / 94.5 eHz（目标 100），RUNNING、faults=0。

波形统计（`motor status` 侧读 + 5 s/20 s RTT CSV 抓取）：

| 抓取文件（`../captures/`） | mean eHz | std | 备注 |
|---|---|---|---|
| `wave_2026-09-08T15-00-04-822Z.csv`（5 s） | 98.46 | 5.29 | 全窗；稳态窗 p1 75.6 / p5 87.9 / median 99.2 / p95 104.1 |
| `wave_2026-09-08T15-01-17-654Z.csv`（5 s） | 98.60 | 6.02 | 稳态窗 p5 89.6 / p95 106.0；<95 约 23% |
| `wave_2026-09-08T15-04-31-523Z.csv`（20 s） | 98.7 | 5.4 | 长窗复核，见 2.3 |

Id/Iq 纹波（20 s 稳态窗）：Id std≈0.019–0.020、Iq std≈0.037–0.043
（零交叉粗估 Iq 主导 ~130 Hz；基线文字记录 ~200–450 Hz，待波形工具精确复核）。
全程无 20 kHz 啸叫。

### 2.3 Speed 通道短时低速凹陷分析（接受现状项）

现象：100 eHz 稳态时 Speed 通道存在 ~2 ms 级短时凹陷（约 23% 时间 <95 eHz，
p5≈89，偶发更低至 ~68），mean/median 仍满足 ±5%。

判别矩阵（均为实测排除）：

| 假设 | 结果 |
|---|---|
| 电流钳位饱和 | 排除：Iq 上限 0.05→0.10（`wave_...15-02-02-594Z.csv`）凹陷依旧，平均 Iq 仅 0.016 |
| 插值/估计假象 | 排除：凹陷为两次编码器样本间的平台；取 AS5600 1 kHz 新样本行统计，<95 仍占 23.1%、<90 4.5% |
| 固定机械/电相位齿槽 | 未发现：自相关仅弱峰 ~19–22 Hz，凹陷电角度无固定相位 |

结论：100 eHz 下 Speed 通道存在真实短时低速凹陷（倾向 AS5600 1 kHz 测速口径与
微速纹波耦合或负载相关），平均/中位满足 ±5%。**接受现状归档**，作为当前装夹的
已知特性；专项延后：同装夹复测定性，必要时评估 `foc_encoder` 滤波/测速时基。
未改 PI、未改滤波参数；4 份原始波形已留档可复现。

## 3. 台架暴露的回归与修复

### 3.1 `encoder cal` 无完成日志（观感失败，实为成功）
旧固件抓拍时打印 `encoder cal: mech/offset`；标定下沉后该日志丢失，只剩
“requested”。修复：provider 抓拍成功分支恢复打印（MODUS 固件）；拒绝路径
带结果码输出（`rejected (%d)`）。

### 3.2 `motor enc` 无反应 = 启动死锁（首次台架暴露的实质缺陷）
根因：20 kHz FOC 引擎由 TIM1 CH4→OC4REF→ADC 注入触发驱动；`haltim1_Stop()`
旧实现 `LL_TIM_DisableAllOutputs` 在急停/停止时连 ADC 触发一起关掉，而 Motor
的 START 命令由 HF ISR 消费——引擎一死，START 永不被执行。
修复（`peripheral/stm32g431/haltim1.c`）：`haltim1_Stop()` 只关三相功率通道
CH1–3（含 N），保留 CH4+MOE，使 HF 引擎在停止/标定结束后持续运行（与 ADC
校准阶段的“只开 CH4、功率全关”安全状态一致）。验证：cal → stop → 再启动 →
反转全部即时可用，空闲态 HF 持续运行（ISR 计数 n≈20 k/s）。

### 3.3 电机路径下波形通道停更（P0-4 审计发现）
Motor 路径下 Iu/Iv/Iw 与 Angle 通道无更新源。修复：`motor_t` 增加最近一拍
相位电流缓存（`qIuLatest/qIvLatest/qIwLatest`），App 侧仅保留 Angle 的 float
换算与 EncMech 镜像；波形通道直接采样 Motor 实时字段。

## 4. 代码收尾（stage-5：foc_app 瘦身）

- Phase 1：删除 legacy 运行路径——App 命令邮箱、legacy 校准/角度/运行步、
  `foc_app_EnterFault/EncoderPoll/SpeedLoop`、`FOC_FAULT_*`、`bMotorControlPath`
  双分支；HF/Clock/Run/Start/Stop/ClearFault/Set*/标定请求全部单路径走 Motor。
- Phase 2：删除镜像字段 `tCore/tCommand/tLifecycle/tCalibration/ptPwmOps/
  ptAdcOps/bMotorControlPath` 与 diagnostics 重复副本；`GetStatus()` 现场从
  `motor_GetStatus/GetFeedback` 组装，波形直读 Motor。
- 结果：RAM −272 B（bss 18920→18648）；foc_app.c 缩减约 500 行。
- 改动文件：`foc/motor/motor.h`、`foc/motor/motor.c`、`foc/app/foc_app.h`、
  `foc/app/foc_app.c`、`peripheral/stm32g431/haltim1.c`、
  `tests/foc/test_motor.c`、`tests/foc/test_foc_minimal_lifecycle.c`（未改）。

## 5. 验证矩阵

| 项 | 命令/范围 | 结果 |
|---|---|---|
| 主机测试 float+fixed | `mingw32-make -C tests/foc minimal CC=gcc` | 全 PASS（core/encoder/lifecycle/motor/as5600；motor 新增标定与 phase-snapshot 用例） |
| 固件 debug-rel | `mingw32-make TARGET_CHIP=stm32g431 BUILD=debug-rel`（clean 与增量均验证） | 通过，text 59940 / data 640 / bss 18648 |
| 固件 debug | clean `BUILD=debug`（收尾前版本） | 通过 |
| 边界 | motor.c 无 as5600/I2C/log/PT；`git diff --check` | 干净 |
| 台架 | 见 §2 | cal×4、100 eHz、反转、停止、故障 0 |

## 6. 遗留/后续（不阻塞）

1. Speed 凹陷定性（§2.3）：同装夹复测；必要时评估 foc_encoder 滤波/测速时基。
2. Id/Iq 纹波频率口径（~130 vs 200–450 Hz）用 mstudio 原始波形复核。
3. 波形 CSV 时间列精度为 1 s（工具侧），频域分析采用行速率近似。
4. 提交/分支操作未执行（工作树未提交），按仓库约定由用户决定。
