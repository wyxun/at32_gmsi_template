# FOC 当前实现优化方案

状态：方案设计已完成并通过复审，待编码实施  
日期：2026-09-10  
适用工程：modus_template  
目标：在不破坏当前 AS5600 速度闭环和已验证功能的前提下，降低 20 kHz 高频路径负担，并改善低速/起步行为。

## 1. 当前基线

当前 STM32G431 的控制入口为：

~~~text
ADC1_2_IRQHandler
└─ foc_app_HighFrequencyISR()
   └─ foc_app_HighFrequencyStep()
      └─ motor_HighFrequencyStep()
         └─ motor_running_step()
            ├─ ADC current sample
            ├─ PositionPort ReadFeedback()
            ├─ speed PI（每 20 拍执行一次）
            ├─ foc_core_step()
            └─ PWM duty commit
~~~

相关实现：

- target/stm32g431/stm32g4xx_it.c：ADC JEOS 中断入口；
- foc/app/foc_app.c：应用层高频入口、周期统计和波形采样；
- foc/motor/motor.c：生命周期、高频调度、位置读取和速度环；
- foc/middleware/foc_core.c：Clarke、Park、PI、反 Park、SVPWM；
- peripheral/driver/as5600.c：AS5600 样本和位置适配；
- foc/observer/foc_encoder.c：角度/速度计算和当前的拍内外推。

20 kHz 周期为 50 μs。STM32G431 在 170 MHz 下，每拍预算约 8500 cycles。当前工程已配置 -O2 优化 FOC 和 Motor 对象，并提供 wIsrMaxCycles 统计；但调试版本还会在每拍执行 mwaveform.Step()。

## 2. 优化目标与非目标

### 2.1 目标

1. 保留当前 FOC 电流环、AS5600 速度闭环、对齐和零位捕获功能。
2. 将 AS5600 的低频采样、编码器状态更新与 20 kHz 控制消费解耦。
3. 解决静止起步时“传感器速度为零，角度外插也为零”的启动问题。
4. 将通用 FOC 数学内核与具体传感器、产品启动策略分层。
5. 用测量结果决定是否优化函数调用、ops 表和调试波形路径，避免凭感觉改动。

### 2.2 非目标

- 不在 foc_core 中加入 AS5600、I²C、启动惯性参数或产品状态机。
- 不在没有实测周期和波形证据时删除安全检查。
- 不把当前 5～30 eHz 的 AS5600 量化限制误判成普通软件性能问题。
- 不为了减少函数层级而破坏 Motor 生命周期、测试接口和多传感器扩展能力。

## 3. 重点方案一：位置生产者/消费者与时间外插

### 3.1 当前问题

AS5600 通过 I²C 以约 1 kHz 的目标频率更新样本，而 motor_read_position() 每次 20 kHz 都会调用位置接口，进一步执行 foc_encoder_Step()。因此当前 20 kHz 消费路径实际包含：

~~~text
读取缓存 → 样本序号判断 → 原始角度差分 → 速度滤波 → 角度外插
~~~

这使传感器观测器的状态更新落在高频 ISR 中。静止时，速度接近零，外插角度自然保持不变；这对测量是正确的，但不能单独承担电机启动时的旋转角生成。

### 3.2 目标数据流

~~~text
约 1 ms 软定时的产品/传感器生产者
└─ foc_encoder_Update()
   ├─ 调用已注册的 AS5600 机械角读取接口
   ├─ 记录当前/上一样本 tick
      ├─ 计算机械速度并滤波
      ├─ 计算并保存对应电角度
      └─ 发布一致的编码器状态

20 kHz FOC 消费者
└─ foc_encoder_GetPosition(tNowTick)
   ├─ 检查 valid 标志与超时（O(1) 直接读取已发布的快照，无自旋轮询）
   ├─ 计算当前控制 tick 与样本 tick 的差值 (tNowTick - tSampleTick + tSensorLatencyTicks)
   ├─ 在 encoder 内按机械速度外插当前机械角度
   └─ 在 encoder 内输出当前电气角度
~~~

建议快照至少包含：

~~~c
uint32_t         wSampleSequence; /* optional sample counter; not ISR seqlock */
foc_time_tick_t  tSampleTick;
foc_angle_t      tMechanicalAngleAtSample;
foc_scalar_t     qMechanicalSpeed;
foc_angle_t      tElectricalAngleAtSample;
bool             bValid;
~~~

`foc_encoder_t` 内部保存上述状态。1 ms 生产向共享状态发布数据时，采用极短临界区（关中断仅持续数条对齐写入指令）或双缓冲原子翻转；20 kHz 消费者始终常数时间 O(1) 读取已发布的稳定快照，严禁在 ISR 中进行自旋重试或循环等待。`wSampleSequence` 如保留，仅用于样本计数/诊断，不作为 ISR 一致性轮询条件。高分辨率 tick 差值在有效外插窗口内使用 64 位有符号数计算。

### 3.2.1 软定时与 I²C 兼容性评估

位置生产不需要与 PWM 边沿严格同步，因此统一放在 foc_app_Run() 的传感器软定时路径是合理的。1 ms 是目标周期，不是硬实时承诺；`foc_encoder_Update()` 通过已注册的位置源接口读取 AS5600，读取成功返回后立即记录完成时刻为内部 `tSampleTick`，Motor 不参与传感器读取。

该方案同时兼容硬件 I²C 和软件 I²C：

- 硬件 I²C：已注册的 AS5600 source 发起一次读取，成功获得机械角后由 encoder 更新；
- 软件 I²C：同一 source 路径执行 GPIO 时序，成功获得机械角后由 encoder 更新；
- 20 kHz ADC/PWM ISR：不等待 I²C、不访问 I²C、不执行阻塞传输。

软定时不代表可以无条件阻塞主循环。软件 I²C 的实现必须满足：

- 不在完整 I²C 事务期间长时间关闭中断；
- 不使用超过高频控制容忍范围的临界区；
- I²C 延迟或失败时不伪造新样本，sequence 只在成功采样后递增；
- 任务被高优先级 ISR 延迟后，按实际 `tSampleTick` 计算 dt；
- 不因一次延迟而连续补执行多次 I²C 读取，避免后台任务形成拥塞。

建议使用“到期后执行一次”的软调度：到达 1 ms 周期就尝试一次采样，采样完成时记录实际时间；如果软件 I²C 导致本次超时，则保留旧快照并进入样本年龄/错误计数处理。位置闭环的实时性由 20 kHz 消费侧的时间外插保证，而不是由 1 ms 软定时保证。

因此，1 ms 软定时是本方案的统一兼容接口；硬件 I²C 和软件 I²C 只属于 AS5600 driver 实现差异，不应改变 `foc_encoder`、foc_core 或 Motor 高频接口。

### 3.3 外插公式

~~~text
Δt = (tNowTick - tSampleTick + tSensorLatencyTicks) / timer_frequency_hz
θmechanical_now = θmechanical_sample + ωmechanical × Δt
~~~

其中 `tSampleTick` 必须是样本实际完成的时间，而不是机械地假设每次恰好 1 ms；`tSensorLatencyTicks` 默认可以为 0，配置后用于补偿传感器群延迟和通信延迟。这样可以吸收 I²C 轮询抖动并保留延迟补偿入口。

### 3.3.1 高速回绕与别名保护

高速转动时需要区分两种情况：

1. BAM32 角度从接近 1 圈回到 0 圈：这是正常的模 2^32 回绕，不是套圈故障；
2. 两次传感器样本之间实际转过了超过半圈：这是采样别名，无法仅凭两个绝对角度判断真实运动方向和圈数。

FOC 角度应使用模运算保存和计算：

~~~text
θmechanical_now = wrap_BAM32(
    θmechanical_sample + Δturns × 2^32
)
~~~

对于当前 BAM32 类型，使用 32 位无符号加法自然回绕即可；机械角度不应使用普通有符号整数累加后再直接比较。机械速度差分仍应使用回绕安全的最短差值。

AS5600 原始角度差分当前按最短路径处理，因此必须满足：

~~~text
|mechanical_speed| × sample_interval < 0.5 turn
~~~

如果超过这个条件，12 位绝对角度样本会发生别名，单靠 AS5600 无法恢复真实圈数。此时应提高采样频率、更换高速传感器，或使用增量编码器/观测器辅助，不能靠继续放大外插解决。

以实际成功样本之间的最大时间间隔 Tgap 计算：

~~~text
最短差分安全上限：机械转速 < 0.5 / Tgap 转/秒
                  < 30 / Tgap(rpm)

整圈混叠上限：    机械转速 = 1 / Tgap 转/秒
                  = 60 / Tgap(rpm)
~~~

当 Tgap = 1 ms 时：

| 场景 | 机械转速 | 说明 |
|---|---:|---|
| 低于 180°/样本 | < 30,000 rpm | 最短差分仍能判断方向 |
| 超过 180°/样本 | > 30,000 rpm | 方向/速度差分开始产生歧义 |
| 约 360°/样本 | 60,000 rpm | 实际 360.1° 可能被读成 0.1° |

用电气频率表示时，极对数为 Pp：

~~~text
180°/样本： eHz = 0.5 × Pp / Tgap
360°/样本： eHz = Pp / Tgap
~~~

上述阈值必须使用实际最大采样间隔，而不是名义 1 ms。若软件 I²C 或后台调度使 Tgap 增大，安全转速按比例下降。例如 Tgap = 2 ms 时，180°歧义从 30,000 rpm 降为 15,000 rpm，整圈混叠从 60,000 rpm 降为 30,000 rpm。

如果系统有可靠的先验速度/方向，可以用“最接近预测角”的多圈解包代替简单最短差分；但当实际已经发生 360.1° 而没有可靠预测时，AS5600 的单圈绝对角度本身无法恢复多出的整圈。

外插还必须限制最大样本年龄：

- `tSampleTick` 回绕使用时间基准规定的安全差值；
- dt 超过有效超时时间时，位置置为无效；
- dt 在有效范围内也应设置最大外插窗口；
- 电气角换算使用足够宽的中间类型，最后再转换为 BAM32；
- 速度突然异常或样本间隔过大时，不应继续盲目外插。

因此，正常的角度 0→接近 2^32→0 回绕不会造成问题；真正的高速限制是传感器采样频率、I²C 延迟和样本间角度别名。

当前 foc_encoder.c 已有基于 hwTicksSinceSample 的拍内外插逻辑。后续应将其拆分为“通用估算算法”和“AS5600/产品适配策略”两部分。重构时应保持以下行为不变：

- 新样本到达时重新锚定实际角度；
- 速度滤波参数和低速不外插门限保持一致；
- 样本超时或磁铁异常时输出无效；
- 停机、对齐和零位捕获后重置观测器基准。

### 3.4 实现放置位置

AS5600 专用的 I²C 细节不放入 foc_core，也不让底层驱动反向依赖 Motor。推荐把 `foc_encoder` 定义为“通用编码器状态与角度估算模块”：AS5600 只提供 `Init` 和 `ReadMechanicalAngle` 两个物理函数；板级/产品组合层把这两个函数和 `as5600_t` 上下文注册到 `foc_encoder`，encoder 初始化时调用 `Init`，传感器软定时调用 `Update()` 时调用 `ReadMechanicalAngle`。encoder 统一维护时间、速度、机械角和电角度状态。

~~~text
foc_core
  只做 Clarke / Park / PI / 反 Park / SVPWM
  不知道 AS5600、I²C、采样频率和启动惯性

foc/observer/foc_encoder.c/h（通用 FOC 库模块）
  注册机械角读取函数和 source context
  接收 tSampleTick，主动读取一次已注册的位置源
  持有上一 tick、速度滤波、机械角和电角度状态
  提供 Update / GetPosition API
  在 encoder 内完成 20 kHz 时刻的角度外插
  不包含 AS5600、I²C 和产品启动策略

peripheral/driver/as5600.c/h
  只实现 AS5600 硬件初始化和机械角度读取
  不保存 sample、sequence、tSampleTick、速度或电角度
  不包含 foc_encoder.h、motor_position.h、motor_params_t

Motor / foc_app / board 组合层
  foc_app/board 持有独立的 AS5600 driver 实例，并将 Init/ReadMechanicalAngle 注册到 foc_encoder
  Motor 持有/配置 foc_encoder，但不持有具体 AS5600 类型
  foc_app/sensor task 低频调度 foc_encoder_Update()
  提供本电机的极对数、方向和 electrical zero
  在 20 kHz 调用 foc_encoder 获取当前机械/电气角度
  负责 ALIGN、启动强制角、惯性参数和传感器接管
  负责选择 encoder 角度或启动强制角
~~~

建议将接口压缩为以下几组，不再为 AS5600 增加 sample/provider/adapter 结构体：

~~~c
typedef foc_result_t (*foc_encoder_source_init_fn)(void *pContext);
typedef foc_result_t (*foc_encoder_read_mechanical_angle_fn)(
    void *pContext, foc_angle_t *ptMechanicalAngle);

/* 产品/板级初始化：将编码器的两个物理 API 注册到 encoder。 */
foc_encoder_RegisterSource(ptEncoder,
                           fnInitSource,
                           fnReadMechanicalAngle,
                           pContext);

/* foc_encoder_Init() 内部调用一次 fnInitSource。 */
foc_encoder_Init(ptEncoder, &tEncoderParams, chPolePairs);

/* Update() 内部调用 fnReadMechanicalAngle。 */
foc_encoder_Update(ptEncoder);

/* 统一位置结果：encoder、无感 observer、Hall 都可提供。 */
typedef struct {
    foc_angle_t  tMechanicalAngle;
    foc_scalar_t qMechanicalSpeed;
    foc_angle_t  tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool         bValid;
} foc_position_t;

/* Motor 在 20 kHz 中调用：从当前选中的位置源得到完整位置。 */
foc_encoder_GetPosition(ptEncoder, tNowTick, &tPosition);
motor_GetPosition(ptMotor, tNowTick, &tPosition);

/* encoder 只保存位置变换所需的最小参数。 */
foc_encoder_SetTransform(ptEncoder, chPolePairs,
                         bDirectionInverted, tElectricalZero);
~~~

这里“在 encoder 中抽象换算”是为了避免每个 Motor/产品重复实现角度乘极对数、方向和回绕；不表示 encoder 拥有完整 `motor_params_t` 或启动状态。Motor 仍是极对数、方向和电气零位的配置来源，encoder 只保存这三个位置变换字段的运行副本。`foc_encoder` 不应保存电阻、电感、磁链等电机控制参数。

`motor_GetPosition()` 是 Motor 对外唯一的位置查询入口，位置可以来自 `foc_encoder`、无感 observer 或 Hall 后端。当前 AS5600 方案使用 `foc_encoder_Update()`；以后替换位置源时，不改变 Motor 和 foc_core 的位置结果接口。

当前工程的 motor_position.h 把位置源拆成了 ops、context 和 feedback 三层；目标方案将其收敛为一个统一的 `foc_position_t` 和一个 `motor_GetPosition()` 入口。不要把传感器快照复制进 foc_core_state_t，也不要让 foc_core 维护 AS5600 专用状态。

### 3.4.1 当前代码的分层问题

当前 peripheral/driver/as5600.h 中的 as5600_sensor_t 同时包含：

~~~c
as5600_t      tDriver;
foc_encoder_t tObserver;
motor_params_t tMotorParams;
foc_angle_t   tElectricalZero;
~~~

并且 peripheral/driver/as5600.c 中的 as5600_position_ReadFeedback() 会在 20 kHz 消费路径直接调用 foc_encoder_Step()。这说明该文件实际同时承担了：

- AS5600 底层 I²C 驱动；
- 编码器观测器；
- 极对数/方向/电气零位转换；
- Motor PositionPort 适配。

因此 as5600_sensor_t 不应继续被命名或理解为“纯 AS5600 driver”。它实际上是一个复合 Position Provider，当前放置位置会让 peripheral/driver 低层依赖 foc/observer 和 foc/motor 的上层接口。

目标结构应改为：

~~~text
peripheral/driver/as5600.h/c
  只保留 as5600_t、Init、ReadMechanicalAngle
  不包含 foc_encoder.h、motor_position.h、motor_params_t

foc/observer/foc_encoder.c/h
  只保存通用 encoder 状态和位置变换参数
  接收 tMechanicalAngle + tSampleTick
  计算机械速度、20 kHz 角度外插和电角度
  提供 GetPosition()，一次返回机械角/速度和电角/电速度

foc_app / board 组合层
  持有独立的 as5600_t 对象
  将 as5600_Init + as5600_ReadMechanicalAngle + &tAs5600 注册到 foc_encoder

Motor
  只持有 foc_encoder_t
  低频调度 foc_encoder_Update()
  高频调用 foc_encoder_GetPosition()
  负责启动状态、零位捕获和故障策略
~~~

依赖方向应明确为：

~~~text
foc_app / board 组合层
  ├─ 持有 as5600_t
  └─ 注册 as5600_ReadMechanicalAngle + context
        ↓
foc_encoder
  Update()
  ├─ 调用已注册的机械角读取函数
  ├─ 保存上一 tick，计算速度、机械角和电角度
  └─ 提供 GetPosition(tNowTick)
        ↑
Motor
  低频调度 Update，20 kHz 查询 GetPosition
~~~

因此 AS5600 driver 不直接调用 `foc_encoder`，也不拥有 sample/provider/observer。AS5600 通过读取函数和 context 注册到 encoder；产品层负责传感器软定时，Motor 只负责 encoder 的参数配置、生命周期和高频位置查询，encoder 集中实现所有与时间、速度和角度相关的状态算法。

重构后，低频 `foc_encoder_Update()` 负责调用已注册 source、记录 tick、更新速度和锚定角度；20 kHz `GetPosition()` 负责按当前 tick 做外插并一次返回机械/电气结果。Motor 仍然提供极对数、方向、电气零位，并决定在启动/故障状态下是否采用当前选中的位置后端输出。

### 3.4.2 不向 AS5600 传递完整电机参数

当前 as5600_sensor_t 保存 motor_params_t，但 AS5600 位置转换实际只使用 chPolePairs；电阻、电感、磁链和参数有效位都与 AS5600 驱动无关。这是过度耦合：

~~~text
motor_params_t
  qResistance
  qInductanceD
  qInductanceQ
  qFlux
  wValidMask
  chPolePairs
~~~

AS5600 driver 不应接收或保存 `motor_params_t`。`foc_encoder` 只保存位置换算所需的三个最小字段，不复制完整 `motor_params_t`。建议按两步收敛：

1. 过渡方案：`foc_encoder` 先稳定提供 `RegisterSource()`、`Update()`、机械速度和机械角外插；
2. 目标方案：`foc_encoder` 增加 `SetTransform(pole_pairs, direction, electrical_zero)` 和 `GetPosition()`；参数由 Motor 从自己的 `motor_params_t` 和运行态零位组装后设置。

如果通用观测器仍需要按电速度决定外插门限，应让它接收明确的电速度门限或最小观测配置，而不是因此保存完整 motor_params_t。观测器的速度滤波和时间外插不需要电阻、电感或磁链参数。

对应的接口方向应为：

~~~text
AS5600 driver
  mechanical angle
       ↓
foc_encoder
  Update()
  调用已注册 source，记录样本 tick
  机械速度、机械角外插、电角度
       ↓
Motor high-frequency path
  调用 GetPosition(tNowTick)
  选择 sensor electrical angle 或 startup forced angle
       ↓
foc_core
  FOC math only
~~~

这样 Motor 仍是电机参数的唯一所有者，`foc_encoder` 只保存位置变换需要的三个字段，传感器驱动只描述传感器自身，通用观测器也不会被电机磁参数污染。

### 3.4.3 明确调用层级

低频生产路径：

~~~text
foc_app_Run()
└─ sensor_BackgroundStep()
   └─ 约 1 ms 软定时到期
      └─ foc_encoder_Update()
         ├─ 调用已注册的 AS5600 机械角读取接口
         ├─ 记录上一 tick/当前 tSampleTick
         ├─ 更新机械速度和滤波状态
         └─ 锚定机械角和电角度
~~~

20 kHz 消费路径：

~~~text
ADC1_2_IRQHandler()
└─ motor_HighFrequencyStep()
   └─ foc_encoder_GetPosition(tNowTick)
      ├─ 根据 tSampleTick 和 tNowTick 外插
      ├─ 输出当前机械角/机械速度
      └─ 输出当前电角度/电角速度
   └─ Motor 选择最终电气角
   └─ foc_core_step()
      └─ 使用电气角完成 FOC
~~~

启动和零位捕获也属于 Motor/产品层：ALIGN 阶段固定控制角度；捕获时 Motor 通过 `motor_GetPosition()` 获取 encoder 输出的机械角度，并根据极对数计算电气零位；随后 STARTUP_OPEN_LOOP 产生强制电气角，满足相位条件后再切换到传感器位置闭环。AS5600 driver 和通用观测器不应知道这些启动策略。

因此现有 motor_position.h 不应作为目标架构继续保留：`motor_position_ops_t`、`motor_position_t` 和 `motor_position_feedback_t` 都只是对 encoder 结果的重复包装。Motor 只保留 `motor_GetPosition()`，内部转发到 `foc_encoder_GetPosition()`；`fnCaptureZero` 由 Motor 对齐流程直接读取 encoder 位置并设置电气零位。

### 3.4.4 传感器到 Motor 的目标调用链

下面是本方案对“传感器数据如何进入 Motor 和 FOC”的目标理解，用于校对职责和调用关系；它不是对当前代码逐项判错。核心原则是：AS5600 只向 `foc_encoder` 提供机械角度，`foc_encoder` 内聚完成时间、速度和机械/电气角度估算，Motor 只查询 encoder 并决定最终控制策略，foc_core 只消费最终电气角度。

#### 低频位置生产链路

~~~text
foc_app_Run()
└─ sensor_BackgroundStep()
   └─ 1 ms 软定时到期
      └─ foc_encoder_Update()
         ├─ 调用已注册的 AS5600 driver
         │  └─ 硬件 I²C/软件 I²C 读取并转换机械角度
         ├─ 保存当前/上一 tick
         ├─ 机械速度差分/滤波
         ├─ 计算机械角与电角度
         └─ 更新 encoder 状态
~~~

该链路只负责产生：

~~~text
机械角度
硬件读取成功/有效状态
~~~

其中 `tSampleTick`、样本序号、样本年龄、机械速度和电气量都由 `foc_encoder` 在收到机械角度后生成或维护，不属于 AS5600 driver 的数据结构。

AS5600 driver 不负责极对数、电气零位和 FOC 启动策略；机械/电气角的通用数学转换和时间外插由 `foc_encoder` 提供，Motor 只负责设置变换参数并决定最终采用时机。

#### 20 kHz Motor 消费链路

~~~text
ADC1_2_IRQHandler()
└─ foc_app_HighFrequencyISR()
   └─ foc_app_HighFrequencyStep()
      └─ motor_HighFrequencyStep()
         └─ motor_running_step()
            ├─ ADC Position：读取本周期电流
            ├─ motor_GetPosition(tNowTick)
            │  └─ foc_encoder_GetPosition(tNowTick)
            │  ├─ 根据 tSampleTick 外插当前机械角度
            │  ├─ 输出机械角度/机械速度/valid
            │  └─ 输出电角度/电角速度
            ├─ Motor 启动策略选择最终电气角度
            │  ├─ ALIGN：固定对齐角
            │  ├─ STARTUP：强制旋转角
            │  ├─ BLEND：强制角与传感器角渐变
            │  └─ CLOSED_LOOP：传感器外插角
            ├─ speed PI / current reference
            ├─ foc_core_step()
            │  ├─ Clarke
            │  ├─ Park（使用 Motor 提供的电气角）
            │  ├─ Id/Iq PI
            │  ├─ inverse Park
            │  └─ SVPWM
            └─ PWM duty commit
~~~

#### 对齐和零位捕获链路

~~~text
Motor 请求 ALIGN
└─ Motor 用固定电气角运行 Id 对齐
   └─ 对齐时间达到
      └─ Motor 通过 motor_GetPosition() 获取当前机械角 snapshot
         ├─ 机械角 × pole pairs
         ├─ 应用方向
         └─ electrical zero = -当前电气角
            └─ Motor 保存 electrical zero
~~~

因此 electrical zero 是 Motor 运行态的控制参数，不属于 AS5600 driver，也不属于通用机械角度观测器。

#### 接口边界

~~~text
AS5600 driver
  mechanical angle
      ↓
foc_encoder
  tick/速度/机械角/电角度估算
      ↓
Motor
  调用 encoder，选择 encoder 角度或启动强制角
      ↓
foc_core
  current control and PWM reference
~~~

初始化阶段由产品/板级组合层把具体读取函数注册到 `foc_encoder`；如果未来换成 ABZ、SPI 磁编码器、BiSS-C 或其他机械角源，只需替换注册的读取函数。Motor 仍只使用 `motor_GetPosition()`，不感知具体传感器。

### 3.4.5 UML 表达

这条链路适合用 UML 表达，建议在评审时同时看三种图：

- 时序图：确认 1 ms 生产和 20 kHz 消费的调用先后；
- 组件图：确认 driver/source ops、foc_encoder、Motor 和 foc_core 的依赖方向；
- 状态图：确认对齐、启动强制角、传感器接管和闭环状态。

#### UML 时序图：传感器生产与 Motor 消费

~~~mermaid
sequenceDiagram
    participant App as foc_app
    participant Bg as sensor_BackgroundStep
    participant Motor as Motor
    participant Drv as AS5600 Driver
    participant Enc as foc_encoder
    participant ISR as ADC1_2_IRQHandler
    participant Core as foc_core
    participant PWM as PWM

    loop 约 1 ms 软定时
        App->>Bg: BackgroundStep()
        Bg->>Enc: Update()
        Enc->>Drv: ReadMechanicalAngle() via registered source
        Drv-->>Enc: tMechanicalAngle
        Enc-->>Bg: update mechanical/electrical state
    end

    loop 每个 PWM 周期（20 kHz）
        ISR->>Motor: HighFrequencyStep()
        Motor->>Enc: GetPosition(tNowTick)
        Enc-->>Motor: mechanical/electrical angle + speed + valid
        Motor->>Motor: select startup or sensor angle
        Motor->>Core: foc_core_step(electrical angle, currents)
        Core-->>Motor: duty
        Motor->>PWM: fnDutyCommit(duty)
    end
~~~

#### UML 组件图：依赖方向

~~~mermaid
flowchart TD
    Driver[AS5600 Driver\nmechanical angle read]
    Encoder[foc_encoder\ntick + observer + angle APIs]
    Motor[Motor/Product Layer\nmotor params + zero + startup policy]
    Core[foc_core\nClarke/Park/PI/SVPWM]

    App[Product/Board Composition
RegisterSource + schedule]
    App -->|RegisterSource(AS5600 callback, context)| Encoder
    Driver -.->|mechanical angle result| Encoder
    Motor -->|GetPosition(tNowTick)| Encoder
    Motor -->|electrical angle + current| Core

    Driver -.->|does not include encoder header| Encoder
    Driver -.->|must not depend on| Motor
    Core -.->|must not depend on| Driver
~~~

#### UML 类图：目标结构体和成员关系

下面的类图是目标结构，不是当前代码的逐字映射；重点用于明确谁拥有数据、谁调用谁，以及哪些成员应该被移除。

~~~mermaid
classDiagram
    class as5600_t {
        +mdi_iic_t* ptIic
        +Init()
        +ReadMechanicalAngle()
    }

    class foc_encoder_params_t {
        +foc_scalar_t qSpeedFilterAlpha
        +uint16_t hwInvalidTimeout
        +uint32_t wSensorLatencyUs
    }

    class foc_encoder_t {
        +foc_encoder_params_t tParams
        +foc_encoder_source_init_fn fnInitSource
        +foc_encoder_read_mechanical_angle_fn fnReadMechanicalAngle
        +void* pSourceContext
        +uint8_t chPolePairs
        +bool bDirectionInverted
        +foc_angle_t tElectricalZero
        +foc_angle_t tMechanicalAngle
        +foc_scalar_t qMechanicalSpeed
        +foc_angle_t tElectricalAngle
        +foc_scalar_t qElectricalSpeed
        +uint32_t wSampleSequence
        +foc_time_tick_t tPreviousSampleTick
        +foc_time_tick_t tSampleTick
        +foc_time_tick_t tSensorLatencyTicks
        +bool bHasSample
        +bool bInitialized
        +bool bValid
        +Init(params, pole_pairs)
        +SetTransform(pole_pairs, direction, zero)
        +RegisterSource(fnInitSource, fnReadMechanicalAngle, pSourceContext)
        +Update()
        +GetPosition(foc_time_tick_t tNowTick)
    }

    class motor_params_t {
        +foc_scalar_t qResistance
        +foc_scalar_t qInductanceD
        +foc_scalar_t qInductanceQ
        +foc_scalar_t qFlux
        +uint32_t wValidMask
        +uint8_t chPolePairs
    }

    class foc_position_t {
        +foc_angle_t tMechanicalAngle
        +foc_scalar_t qMechanicalSpeed
        +foc_angle_t tElectricalAngle
        +foc_scalar_t qElectricalSpeed
        +bool bValid
    }

    class motor_config_runtime_t {
        +motor_params_t tParams
        +foc_encoder_t tEncoder
    }

    class motor_control_runtime_t {
        +foc_core_input_t tCycleInput
        +foc_core_state_t tCore
    }

    class motor_t {
        +motor_config_runtime_t tConfig
        +motor_control_runtime_t tControl
        +GetPosition(foc_time_tick_t tNowTick, foc_position_t* ptPosition)
    }

    foc_encoder_t ..> as5600_t : registered Init/Read APIs
    motor_t --> foc_encoder_t : GetPosition
    foc_encoder_t ..> foc_position_t : returns
    motor_t ..> foc_position_t : GetPosition returns
    motor_config_runtime_t *-- motor_params_t : owns motor data
    motor_config_runtime_t *-- foc_encoder_t : owns encoder
    foc_encoder_t *-- foc_encoder_params_t : owns params
    motor_t *-- motor_config_runtime_t
    motor_t *-- motor_control_runtime_t
~~~

类图中的关键关系：

- `as5600_t` 是产品/板级持有的独立硬件对象，只保存硬件访问句柄，不保存 sample、sequence、tSampleTick、速度或电角度；
- `foc_encoder_t` 集中保存上一/当前 tick、机械角、机械速度、电角度、电角速度和位置变换参数；
- `foc_position_t` 是 encoder、无感 observer、Hall 后端共同使用的统一位置结果；
- `motor_GetPosition()` 是 Motor 对外唯一的位置查询入口，当前转发到 `foc_encoder`，未来可切换其他位置后端；
- `motor_params_t` 只由 Motor 持有，`foc_encoder` 只保存极对数、方向和电气零位这三个位置变换字段；
- `motor_feedback_t` 不再重复携带位置结果，位置通过 `motor_GetPosition()` 单独查询。

#### UML 状态图：启动和传感器接管

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> ALIGN: request align/start
    ALIGN --> CAPTURE_ZERO: alignment complete
    CAPTURE_ZERO --> STARTUP_OPEN_LOOP: zero captured
    STARTUP_OPEN_LOOP --> SENSOR_BLEND: valid samples + phase error acceptable
    SENSOR_BLEND --> SENSOR_CLOSED_LOOP: phase stable for N samples / call foc_pid_Precharge() once before speed PI enable
    SENSOR_BLEND --> STARTUP_OPEN_LOOP: phase error too large
    STARTUP_OPEN_LOOP --> FAULT: timeout/current fault
    SENSOR_BLEND --> FAULT: sensor invalid/timeout
    SENSOR_CLOSED_LOOP --> FAULT: sensor invalid/timeout
    SENSOR_CLOSED_LOOP --> IDLE: stop
    FAULT --> IDLE: clear fault
~~~

UML 图中的关键校验点是：AS5600 Driver 只向 `foc_encoder` 提供机械角度；Motor 不持有或读取 AS5600，只通过 `motor_GetPosition()` 查询 encoder；`foc_encoder` 内聚完成速度、外插和电角度；Motor 决定最终采用 encoder 角度还是启动强制角；foc_core 不反向依赖任何传感器或产品启动状态。

### 3.4.6 按该架构的修改方法

建议按以下顺序实施，保证当前 AS5600 速度闭环行为可回退、可对比：

1. **拆纯 AS5600 driver。**
   `peripheral/driver/as5600.h/c` 只保留一个 `as5600_t`，提供硬件初始化和 `as5600_ReadMechanicalAngle()`。移除 `as5600_sample_t`、`as5600_sensor_t`、sample/sequence/tick 缓存以及对 `foc_encoder.h`、`motor_position.h`、`motor_params.h` 的依赖。

2. **把 AS5600 source 注册到 encoder。**
   不再增加 AS5600 source adapter 结构体；产品/板级组合层完成一次注册：

   ~~~c
   foc_encoder_RegisterSource(&tEncoder,
                              as5600_Init,
                              as5600_ReadMechanicalAngle,
                              &tAs5600);
   ~~~

   这样 AS5600 只有一个硬件对象，tick 的语义完全由 `foc_encoder` 解释和保存。

3. **把完整角度估算内聚到 `foc_encoder_t`。**
   `foc_encoder_t` 统一保存上一 tick、当前 tick、机械角、机械速度、机械角外插、电角度、电角速度和有效状态。低频 `Update()` 锚定新样本，20 kHz `GetPosition()` 只按当前 tick 查询完整估算结果。

4. **让 Motor 成为调用方和参数所有者。**
   产品/板级组合层初始化 AS5600 并注册到 encoder；Motor 只从自己的 `motor_params_t` 设置极对数，并设置方向/电气零位；传感器软定时调用 `Update()`，高频路径调用 `motor_GetPosition(tNowTick)`；Motor 在 ALIGN、STARTUP、BLEND、FAULT 状态下决定是否采用位置后端返回的电气角。

5. **删除 Motor 位置 ops/feedback 中间层。**
   目标上移除 `motor_position_ops_t`、`motor_position_t` 和 `motor_position_feedback_t`。Motor 只保留 `motor_GetPosition(ptMotor, tNowTick, &tPosition)`；当前实现直接转发到 `foc_encoder_GetPosition()`，未来无感 observer 或 Hall 后端也只需提供同一个 `foc_position_t` 结果。`fnCaptureZero` 移回 Motor 的对齐完成流程；它读取 encoder 机械角并由 Motor 使用极对数计算/保存电气零位。

6. **统一位置结果结构体。**
   以 `foc_position_t` 作为所有位置后端的公共结果；`foc_encoder_t` 只负责 encoder 后端的内部状态。删除 `motor_feedback_t.tPosition`，避免位置反馈和电流/电压/PWM 反馈混在一个结构体中。删除重复的 `as5600_sample_t`、`motor_position_feedback_t`、`foc_position_estimate_t` 和 provider 状态，必要时仅在迁移分支保留兼容 typedef。

7. **迁移验证。**
   先在无硬件环境下验证角度回绕、样本 tick、机械速度、20 kHz 外插、极对数换算和电气零位；再用原 AS5600 路径做 A/B 波形对比。没有硬件时不得宣称速度闭环已实测通过；必须保留旧路径或编译开关，确保新接口异常时能快速回退。

### 3.4.7 角度预测的内聚性决策

“由 Motor 在 20 kHz 预测”建议理解为“Motor 在 20 kHz 调用 encoder 查询”，而不是把预测公式复制到 Motor。推荐最终形式是：

~~~text
传感器软定时：
  foc_encoder_Update()
  └─ 已注册 source：AS5600_ReadMechanicalAngle()

Motor 20 kHz：
  motor_GetPosition(tNowTick, &tPosition)
  └─ 当前后端：foc_encoder_GetPosition(tNowTick, &tPosition)
  └─ encoder 内部完成 dt、速度 × dt、回绕和有效性判断
~~~

原因是“上一 tick、当前 tick、样本机械角、机械速度、外插窗口、电角度”本来就是同一条估算链的状态。如果把 `speed × dt` 外插拆到 Motor，Motor 必须了解 encoder 的样本年龄和速度有效性；以后换传感器或调整滤波时，估算逻辑会被拆成两份。保持算法内聚在 `foc_encoder`，只把传感器软定时放在产品层、位置查询放在 Motor，可以同时满足低层驱动简单、Motor 控制策略独立和估算实现可复用。

如果后续切换无感 observer 或 Hall，也不增加 `motor_position_ops_t` 这类多函数接口；只让当前 Motor 位置后端生成统一的 `foc_position_t`，由 `motor_GetPosition()` 对外提供。

如果当前实现确实需要把不同位置后端做成运行时可替换，也只保留 `RegisterSource()` 的 Init/ReadMechanicalAngle 两个函数和 context，不新增 AS5600 sample/provider 结构体。

当前 AS5600 只有一个产品实例时，由产品/板级组合层把 `as5600_ReadMechanicalAngle()` 注册到 `foc_encoder` 最简单；将来扩展无感或 Hall 时，替换位置源注册或后端即可，`motor_GetPosition()` 和 foc_core 接口不变。

### 3.4.8 API 调用语义和命名

最终调用时序应明确为：

~~~text
约 1 kHz 传感器软定时：
  foc_encoder_Update()
  ├─ 调用已注册 source：AS5600_ReadMechanicalAngle()
  └─ 记录 tSampleTick 并更新 encoder 锚点、速度和电角度

20 kHz FOC：
  motor_GetPosition(tNowTick, &position)
  └─ 当前后端为 foc_encoder_GetPosition(tNowTick, &position)
     └─ 按 tNowTick - tSampleTick 外插并返回机械/电气位置
~~~

这里 `GetPosition()` 是“查询当前估算位置”，可以在函数内部完成计算，但不应改变上一有效样本的锚点状态；`Update()` 才是写入新样本、改变估算状态的接口。因此：

- `Update`：适合低频样本输入，含义是更新状态；
- `GetPosition`：适合 20 kHz FOC 查询，含义是返回当前最优估算位置；
- `Step`：只用于确实需要每个 20 kHz 拍推进内部状态的 observer，例如无感 observer；不作为所有位置源的统一公共命名。

在当前 AS5600 方案中，`foc_encoder_Update()` 由 `foc_app` 的传感器软定时路径调用，属于产品层的调度和生命周期管控；AS5600 通过已注册 source 提供机械角，Motor 不直接读取 AS5600。20 kHz 路径只调用 `motor_GetPosition()`/`foc_encoder_GetPosition()`，不重复读取 AS5600，也不执行 `Update()`。

如果 Motor 需要运行时切换 encoder、无感或 Hall，可以只保存一个函数指针，不再恢复 `motor_position_ops_t` 多函数表：

~~~c
typedef foc_result_t (*motor_get_position_fn)(
    void *pContext, foc_time_tick_t tNowTick, foc_position_t *ptPosition);
~~~

初始化 AS5600 方案时，由产品/板级组合层调用 `foc_encoder_RegisterSource()` 绑定 Init/ReadMechanicalAngle 两个函数；encoder 初始化时执行 Init，`Update()` 时执行 ReadMechanicalAngle。Motor 侧只绑定统一的 `motor_GetPosition()` 后端查询入口。切换其他编码器时只替换这两个物理函数，对 Motor 调用者而言始终只有 `motor_GetPosition()`，不会感知底层来源。

### 3.4.9 并发提交、时间基准与传感器延迟

#### 3.4.9.1 不在 20 kHz ISR 中循环等待

`foc_encoder_Update()` 应先在局部变量中完成角度差分、速度滤波、电角度换算和有效性判断，最后只在提交共享状态时进入极短临界区：

~~~text
传感器后台任务：
  调用已注册 AS5600 source（不关中断）
  计算 next encoder state（局部变量）
  disable_irq()
    一次性写入完整 published state
  enable_irq()

20 kHz ISR：
  直接读取 published state
  不循环重试、不等待后台、不访问 I²C
~~~

临界区只包含若干对齐的标量写入，不能包住 I²C 事务、浮点计算或滤波计算。单核 Cortex-M4 上，后台临界区可能被 20 kHz ISR 请求打断；如果 ISR 采用 seqlock 循环等待，而后台正处于被抢占的写入阶段，序号不会变化，ISR 可能死循环。因此本方案禁止“ISR seqlock 轮询”。双缓冲加单字节/32 位 ready index 也可以采用，但必须保证 ISR 永不读取正在写入的 buffer；当前单核裸机实现优先使用极短中断保护，保持 O(1) 消费。

当前 `as5600_GetSample()` 的序号双读不能直接作为目标实现：写者没有明确的写入中状态，读者可能在多个字段写入期间读到序号前后一致但内容不一致的快照。重构为“AS5600 只返回机械角、encoder 一次性提交状态”后，应删除这类缓存快照保护，统一在 encoder 发布点解决并发。

#### 3.4.9.2 使用高分辨率系统时基，不使用 1 ms SysTick 计数

这里要区分三种时间：

~~~text
错误：get_system_ms() 或 SysTick 中断次数
  只有 1 ms 粒度，不能用于 20 kHz 角度外插

正确：get_system_ticks()
  读取 SysTick 当前计数值 + 溢出累计值
  单位是系统时钟 tick；170 MHz 时约 5.88 ns/tick

可选：get_system_us()
  由高分辨率 tick 换算得到微秒
  适合配置/日志，但会丢掉亚微秒信息
~~~

位置接口的内部规范应使用 `get_system_ticks()` 同源的高分辨率时基：

~~~c
typedef int64_t foc_time_tick_t;

foc_encoder_Update(ptEncoder);
foc_encoder_GetPosition(ptEncoder, tNowTick, &tPosition);
~~~

`tSampleTick` 必须在 AS5600 I²C 读取成功完成后记录；不能使用只在 1 ms 任务入口记录的时间，也不能用 20 kHz 调用次数代替实际样本时间。`tNowTick - tSampleTick` 使用有符号 64 位差值，实际有效窗口只有几毫秒，因此不会接近 `int64_t` 回绕。

当前 perf_counter 的 `get_system_ticks()` 已经把 SysTick 当前值和溢出累计值组合成高分辨率时间；不要直接读取 `SysTick->VAL` 作为完整时间戳，也不要只读取 SysTick ISR 的 1 ms 计数。若 20 kHz 中调用 `get_system_ticks()` 的临界区和函数开销经周期测量过高，再由 target 提供等价的低开销 `foc_time_now_tick()`。

微秒只用于人可读配置和传感器延迟参数。初始化时将 `wSensorLatencyUs` 转换为同一时基的 `tSensorLatencyTicks`；运行时统一计算：

~~~text
tDtTicks = (tNowTick - tSampleTick) + tSensorLatencyTicks
tDtSeconds = tDtTicks / timer_frequency_hz
~~~

`speed × tDtTicks` 使用明确的宽中间类型，并限制 `tDtTicks` 在有效外插窗口内。这样既保留 SysTick 的实际亚微秒精度，又不把 170 MHz 原始 tick 直接暴露为各模块都要理解的单位。

#### 3.4.9.3 传感器延迟补偿

`foc_encoder_params_t` 增加有效配置项：

~~~c
uint32_t wSensorLatencyUs;
~~~

它表示“传感器内部群延迟 + 从实际磁场采样时刻到 I²C 读取完成时刻的有效延迟”，不是未经测量就固定写死的 AS5600 常数。当前外插时间定义为：

~~~text
tDtTicks = (tNowTick - tSampleTick) + tSensorLatencyTicks
θmechanical_now = θmechanical_sample + ωmechanical × (tDtTicks / timer_frequency_hz)
~~~

延迟补偿只在样本有效、速度有效且未超过最大外插窗口时启用；`wSensorLatencyUs` 默认应为 0，待示波器/波形或已知转速相位差测量后配置。以 100 eHz、300 μs 为例，补偿量约为 10.8° 电角度，这个量级足以影响 Id/Iq 解耦和最大扭矩，必须纳入硬件验收。

### 3.5 低速和量化限制

外插只能减小采样保持造成的台阶，不能创造新的传感器信息。AS5600 12 位量化在 5～30 eHz 区间仍然会造成明显速度抖动，因此低速门限、速度滤波和超时保护必须保留。必要时使用门限滞回，避免在门限附近频繁切换“外插/不外插”。

## 4. 重点方案二：静止起步、惯性和传感器接管

### 4.1 为什么静止时不能只靠传感器外插

对齐完成后，若电气角度保持 0，且只施加 Iq：

~~~text
ωsensor ≈ 0
→ 外插增量 ≈ 0
→ 定子磁场角度不旋转
→ 转子可能只被拉到一个平衡位置
→ 不能保证持续起转
~~~

所以，启动阶段必须有独立于传感器瞬时速度的旋转角生成机制。

### 4.2 推荐启动状态机

~~~text
IDLE
  ↓
ALIGN
  固定电角度，施加 Id，等待转子稳定
  ↓
CAPTURE_ZERO
  捕获电气零位并重置位置估算基准
  ↓
STARTUP_OPEN_LOOP
  由产品层生成递增的强制电气角度，逐步增加角速度/Iq
  ↓
SENSOR_BLEND
  比较强制角与传感器估计角，满足条件后逐步切换
  ↓
SENSOR_CLOSED_LOOP
  由传感器快照和时间外插角度驱动 FOC
~~~

启动强制角的基本形式为：

~~~text
θforced[k+1] = θforced[k] + ωstartup[k] × Ts
~~~

其中 ωstartup 的斜率、最大值和 Iq 电流斜坡应作为产品参数。不同负载和转动惯量使用不同参数，不能写死在通用 FOC 内核。

### 4.3 传感器接管条件

不建议仅凭“AS5600 已有有效样本”就立即切换。建议至少同时检查：

- 样本没有超时，磁铁状态正常；
- 传感器速度达到可用区间，或已经连续获得多个有效样本；
- 强制角和传感器角的包络相位差小于设定阈值；
- 相位差在连续若干个采样周期内稳定；
- 接管过程中有角度渐变/混合，不能瞬间跳变。

若启动超时或相位误差持续过大，应停止 PWM 并进入故障或重新对齐，而不是继续盲目加速。

### 4.4 速度环无扰切换（Bumpless Transfer）

从 `STARTUP_OPEN_LOOP`/`SENSOR_BLEND` 进入速度闭环时，不能只渐变角度而让速度 PI 的积分状态仍为 0。应在第一次启用速度 PI 之前，用当前实际施加的开环 `Iq` 输出预充积分器，使 PI 总输出连续。

若速度 PI 包含比例和微分项，推荐预充公式为：

~~~text
e       = speed_reference - speed_feedback
uP      = Kp × e
uD      = Kd × (e - previous_e) / Ts
integrator = clamp(Iq_open_loop - uP - uD)
previous_e = e
~~~

如果当前速度 PI 的微分项为 0，公式退化为 `integrator = clamp(Iq_open_loop - uP)`；不能简单把积分器直接设置成 `Iq_open_loop`，否则比例项仍会叠加出转矩阶跃。预充值还必须使用已经限幅的实际开环 `Iq`，并受积分器上下限约束。

建议新增独立的 `foc_pid_Precharge()` API，而不是由 Motor 直接修改 PI 结构体成员：

~~~c
foc_result_t foc_pid_Precharge(foc_pid_t *ptPid,
                               foc_scalar_t qManualOutput,
                               foc_scalar_t qReference,
                               foc_scalar_t qFeedback);
~~~

该 API 应在 `SENSOR_BLEND` 确认完成、即将进入 `SENSOR_CLOSED_LOOP` 并首次启用速度 PI 之前调用一次；如果 `SENSOR_BLEND` 阶段已经启用速度 PI，则改为在进入 `SENSOR_BLEND` 前调用一次。随后速度 PI 按正常周期运行。验收时重点观察切换前后 `Iq reference`、速度 PI 输出和实际 Iq 是否连续，以及是否出现速度超调或过流保护。

## 5. 模块职责边界

~~~text
传感器驱动层
  AS5600 I²C、机械角度读取和硬件状态

通用位置估算层
  foc_encoder：保存 tSampleTick、速度滤波、机械角/速度、电角/电速度
  负责时间外插、有效性/超时和角度回绕

Motor/产品层
  读取传感器机械角并把当前 tick 传给 foc_encoder
  提供 pole pairs、方向和 electrical zero
  负责 ALIGN、零位捕获、启动强制角、惯性参数、传感器接管

foc_core
  Clarke、Park、电流 PI、反 Park、SVPWM
  只接收已经准备好的电气角度和电流输入
~~~

因此，时间戳外插、速度估算和机械角到电气角的纯数学部分统一内聚在 FOC 库内 `foc_encoder`；AS5600 的 I²C、量化和硬件状态留在 driver，产品启动策略留在 Motor 产品层。`foc_encoder` 只依赖抽象的机械角读取回调，不依赖具体 AS5600 类型或 I²C 实现。

## 6. 其他当前 20 kHz 路径优化点

### 6.1 调用层级

20 kHz ISR 只保留固定且短的调用链：

~~~text
ADC ISR
  └─ foc_app_HighFrequencyStep()
      └─ motor_HighFrequencyStep()
          ├─ motor_GetPosition()
          │   └─ foc_encoder_GetPosition()
          └─ foc_core_step()
~~~

`foc_app_HighFrequencyStep()` 只做入口转发、一次 tick 读取和必要的
周期统计；`motor_HighFrequencyStep()` 负责生命周期分发；
`motor_GetPosition()` 只能是薄转发，不得复制 feedback、查询诊断、
访问 I²C 或再套一层 PositionPort。`foc_encoder_GetPosition()` 只能
读取已发布状态，不得向下调用 driver、source poll 或多层 ops。

高频位置查询不得经过函数指针；函数指针只允许出现在低频 source
初始化和采样更新路径。只有在最终 ELF 反汇编确认出现明显 BL/BLX
且有周期收益时，才考虑固定芯片的 inline/直接绑定。源码层合并不会
减少 Clarke、Park、PI、SVPWM 等计算量，不能以增加调用层包装换取所谓
抽象性。

### 6.2 ops 函数指针

当前 ADC、Position、PWM 回调通过 ops 表访问。-> 本身只是成员地址计算和加载，真正的额外跳转是函数指针调用。建议顺序：

1. 保留通用 ops 接口；
2. 用最终 ELF 反汇编确认是否产生 BLX；
3. 仅对固定产品目标增加可选的直接绑定/inline 路径；
4. 不改变通用测试和传感器替换接口。

### 6.3 调试波形

当前 float 调试配置每个高频周期调用 mwaveform.Step()。它不是 FOC 必需计算，可能包含通道遍历、浮点缩放、内存复制和 FIFO 写入。

建议：

- 生产版本保持编译关闭；
- 调试版本支持固定抽取率，例如 20 kHz → 1 kHz；
- 将完整波形打包尽量放到后台，ISR 只写最小采样记录；
- 统计 mwaveform.Step() 单独占用的 cycles。

### 6.4 foc_core_step() 内部

当前 foc_core_step() 中 Clarke、Park、一次 sin/cos、缓存 Park/反 Park、电流 PI 和 SVPWM 均有实际用途，不应直接删除。可评估的低风险优化是：

- 外部 API 保留参数检查；
- 内部增加一次校验后的 static inline fast path；
- 电压模式只有在确认不需要电流观测/保护时，才跳过不参与计算的电流变换；
- 保留 sin/cos 一次计算并由 Park、反 Park 复用。

### 6.5 ADC 中断健壮性

当前 ADC ISR 清除 JEOS 和 OVR 后直接进入高频控制。后续应确认：

- ADC1/ADC2 的 JEOS 触发关系是否与双 ADC 配置一致；
- OVR 清除前是否需要记录故障计数；
- 完整 ISR 时间是否包含标志位处理、异常进出和应用高频路径。

这部分属于目标芯片适配层，不应放入 foc_core。

## 7. 分阶段实施计划

### 阶段 A：无硬件先做

- 增加带时间戳的编码器估算器 host 测试；
- 增加高分辨率 raw tick 的回绕、差值、延迟补偿和最大外插窗口测试；
- 测试角度回绕、正反转、样本抖动、丢样本和超时；
- 用临界区测试桩验证发布状态不会使用 ISR seqlock 循环等待；
- 测试静止、启动强制角、传感器接管和相位误差判断；
- 保持 float/fixed 双后端测试；
- 编译 debug-rel/release，检查符号和反汇编；
- 将当前实测限制和预期波形写入测试记录。

### 阶段 B：位置生产者/消费者重构

- 统一 `tSampleTick/tNowTick` 为 `get_system_ticks()` 同源的高分辨率时间基准；
- `foc_encoder_Update()` 在传感器后台通过已注册 source 读取机械角，记录完成时刻为 `tSampleTick`，计算局部状态并以极短临界区发布；
- 20 kHz 只调用 `foc_encoder_GetPosition(tNowTick)`，不循环重试、不访问 I²C；
- 增加 `wSensorLatencyUs`，默认 0，硬件测量后再配置；
- 保留旧路径作为可切换回退方案；
- 先不改变 AS5600 I²C 采样周期和滤波参数。

### 阶段 C：启动策略

- 增加启动子状态或 MOTOR_STATE_STARTUP；
- 实现强制角速度斜坡和 Iq 斜坡；
- 增加传感器接管条件和相位差保护；
- 增加速度 PI 无扰预充，保证开环 Iq 到速度闭环输出连续；
- 对齐完成后重置生产者/消费者时间基准；
- 无硬件时完成状态机和边界测试，暂不宣称启动性能已验证。

### 阶段 D：高频路径优化

- 用 cycle counter 分别测量 ADC、位置读取、FOC core、PWM、波形；
- 根据反汇编决定是否处理 BL/BLX；
- 优化或抽取 mwaveform.Step()；
- 再评估 checked API/fast path 和电压模式快速路径。

### 阶段 E：硬件验证

- 空载和不同惯性负载启动；
- 低速 5～30 eHz、平滑区 50+ eHz、100 eHz 上限；
- 对齐重复性、零位相位误差、起步失败率；
- ISR 最大 cycles、抖动、PWM 更新时序和 ADC OVR；
- 确认当前已实测的 12 V / 100 eHz 能力不被破坏。

## 8. 验收标准

### 功能

- 对齐、零位捕获、AS5600 速度闭环和停止/故障行为不退化；
- 静止起步不依赖 AS5600 速度先变为非零；
- 传感器接管无明显电角度跳变；
- 传感器失效或超时可安全停机。
- `motor_GetPosition()` 可统一返回 encoder、无感或 Hall 后端的 `foc_position_t`。

### 性能

- 20 kHz 高频路径在目标周期预算内；
- 位置消费者不再执行完整 foc_encoder_Step() 状态更新；
- 调试波形不会成为生产控制路径的隐性负担；
- 优化前后用同一组输入和反汇编/周期数据比较。
- 20 kHz 不得因等待后台 seqlock 而出现不可界定的循环时间。

### 兼容性

- foc_core 仍不依赖具体传感器；
- AS5600 只是 `foc_encoder` 注册的原始角度读取实现；
- 后续可接入霍尔、增量编码器或无感观测器；
- host 测试和现有生命周期契约保持通过。
- 时间戳回绕、传感器延迟补偿和速度环无扰切换均有独立测试项。

## 9. 当前明确结论

1. 20 kHz 位置读取应改成“读取快照 + 按时间外插”，但外插不是静止起步机制。
2. 静止起步必须由 Motor/产品层提供强制旋转角和惯性相关的启动斜坡。
3. AS5600 驱动、通用位置估算、启动策略和 foc_core 应保持职责分离。
4. foc_core_step() 的数学主流程目前没有整体冗余，优先优化调用边界和调试路径。
5. 共享 encoder 状态禁止在 20 kHz ISR 中 seqlock 循环重试；后台只在极短临界区提交完整状态。
6. 位置接口内部统一使用 `get_system_ticks()` 同源的高分辨率系统 tick；微秒仅作为传感器延迟等人可读配置单位，并在初始化时转换为 tick。
7. 开环切闭环必须同时处理角度渐变和速度 PI 无扰预充。
8. 所有代码优化必须先保留可回退路径，并以 host 测试、反汇编和后续硬件实测作为依据。
