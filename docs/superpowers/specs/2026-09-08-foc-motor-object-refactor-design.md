# FOC Motor 对象重构设计草案

> 日期：2026-09-08  
> 状态：已获准进入实施；独立 Motor、Core 去重、AS5600 provider，以及三种控制模式 wrapper 已落地。编码器零位对齐已于 2026-09-09 下沉到 Motor（`MOTOR_STATE_POSITION_CAL` + `motor_RequestPositionCalibration()`，App 位置 provider 实现 `fnCaptureElectricalZero`，旧 App 标定服务删除），主机测试与 G431 固件构建通过，待台架回归确认 offset 重复性与速度闭环基线。

## 1. 目标

将当前 `foc_app_t` 中与电机控制直接相关的成员和行为抽离为独立的
`motor_t`。`foc_app_t` 只保留 MODUS 对象接入、Shell、波形和产品级调度，
`motor_t` 成为可单独初始化、调度和主机测试的单电机控制对象。

本次重构保持以下约束：

- 单电机、静态内存、无动态分配。
- 一个 `motor_t` 在初始化时只注册一个活动位置源。
- 不在通用框架中实现位置源资格判定、在线切换、无扰混合或双编码器一致性。
- 双编码器或 HFI+SMO 等组合需求由一个具体的复合位置源自行实现。
- 高频路径不记录日志、不访问 I2C、不阻塞。
- ADC、PWM 和位置源继续通过接口注入，通用 FOC 代码不包含芯片 HAL。
- 所有硬件和算法接口均携带实例上下文，不依赖隐藏全局状态；当前单电机产品
  可以使用单例上下文，但基础接口不把单例写死。
- 轻量化不仅是缩小 `foc_app_t`，还必须避免配置、输入、反馈和诊断状态的
  重复保存；Motor 内部只保留一份可变运行状态。
- 不因为抽象而增加通用注册表、事件总线、控制器管理器或独立 Mailbox
  Mdriver；扩展点必须保持足够小，且不改变高频主流程。

## 2. 当前代码结论

当前 `foc_encoder_params_t tEncoderParams` 不是死成员：

1. `foc_app` 校验它的极对数、超时、周期和滤波系数。
2. `chPolePairs` 被复制到 `foc_app_position_t`，用于机械量到电气量的换算。
3. 未显式注入传感器时，整组参数经 `foc_port_SensorInit()` 传入
   `as5600_sensor_Init()`。
4. `as5600_sensor_Init()` 再调用 `foc_encoder_Init()`，滤波、超时和高频周期
   最终由编码器观测器使用。

问题在于该结构混合了三类配置：

- `chPolePairs` 属于电机本体。
- `qHighFrequencyPeriod` 属于控制时基。
- `qSpeedFilterAlpha`、`hwInvalidTimeout` 属于 AS5600/编码器位置源。

现有高级观察器源码仍依赖已经不存在的 `motor_position.h`，且未进入
`foc/foc.mk` 的实际构建清单。因此本次不直接恢复旧位置源框架，而是先建立
最小的新接口，再逐个迁移需要启用的算法。

当前 ADC 零偏由 `foc_app_Start()` 调用 `fnCalibrationBegin()`，之后在高频
中断中逐拍执行 `fnCalibrationStep()`；也就是说目前每次 Start 都重新校准。
STM32G431 端已有 `haltim1_StartAdcTrigger()`，可以在三相功率输出关闭时只启动
CH4 ADC 触发。因此把零偏初始化前移到 Motor 的首次高频调度具有现成硬件
基础，但这是一次明确的生命周期行为调整，需要用测试锁定。

## 3. 方案比较

### 方案 A：单活动位置源的组合式 Motor 对象（推荐）

`motor_t` 持有一个 `motor_position_t` 接口。初始化时选择 AS5600、Hall、
SMO、NLFO 或某个复合位置源，运行时不再切换。

优点：对象边界清楚、状态量少、ISR 路径固定、主机测试容易。新增位置算法
只新增 provider，不修改 `motor.c`。

代价：如果产品确实需要运行中切换，切换逻辑需要封装成一个复合 provider。

### 方案 B：Motor 内置多位置源管理器

`motor_t` 保存多个位置源，统一处理置信度、资格判定、主备切换和混合。

优点：适合需要低速 HFI、中高速 SMO、编码器故障降级的复杂产品。

代价：生命周期、故障语义、参数和测试矩阵明显膨胀，不符合当前精简目标。

### 方案 C：编译期枚举加联合体

使用枚举决定 AS5600、Hall 或 SMO，并用联合体保存具体实例，不使用 ops。

优点：调用链最直接，便于编译器内联。

代价：每增加位置源都要修改 `motor.c`，通用层重新依赖所有具体算法和驱动，
不利于跨芯片复用。

本设计采用方案 A。

## 3a. 实测记录（2026-09-07，motor enc 0.05 100）

以下实测结果作为本次对象迁移的行为基线。重构后的 AS5600 provider、位置
换算、速度 PI 和波形输出必须保持同一数值口径；若结果变化，必须能够明确
归因于已经评审的生命周期调整，而不能由对象拆分或接口迁移引入隐性变化。

| 项 | 实测 |
|---|---|
| cal offset 重复性 | 0.4597/0.4630/0.4648/0.4614，极差 0.0051 turn |
| cal 对齐电流 | Id ref 0.10 → actual 0.110，Vd=0.108（电流环跟踪） |
| Speed（5s 波形） | mean 98.5 eHz，std 4.9（±5%），范围 95-105 |
| Id / Iq | ≈0 / ±0.06（残留 ±0.2 纹波，约 200-450 Hz，无高频声） |
| 100 eHz 上限 | 12V 可稳定跑（BEMF≈2.1V）；此前“48 eHz”作废 |

已知限制必须保留在验收说明中：5-30 eHz 低速不平滑来自 AS5600 1 kHz
量化，量化误差约为 ±1.7 eHz；50+ eHz 时量化误差低于 5%。Speed 波形的
1 kHz 更新台阶属于正常现象，不得为了“抹平”台阶而在 20 kHz 高频路径加入
I2C 访问、阻塞等待或不受控的插值状态。

## 4. 总体结构

```text
foc_app_t
├── modus_base_t / RingBuffer
├── Shell、波形和产品诊断
└── motor_t
    ├── motor_params_t              唯一电机物理参数
    ├── foc_adc_ops_t * + context   ADC 采样与偏置校准
    ├── foc_pwm_ops_t * + context   duty、使能和急停
    ├── motor_position_t            唯一活动位置源
    ├── foc_core_state_t            收缩后的电流环工作状态
    ├── foc_pid_t                   速度环运行状态
    ├── foc_adc_calib_t             ADC 偏置状态
    ├── motor_command_sync_t        前台到高频入口的最小同步状态
    ├── foc_core_command_t          当前生效控制命令
    └── motor_lifecycle_t           初始化、运行和故障状态
```

`motor_feedback_t` 和 `motor_status_t` 都是查询 API 的输出类型，不作为 `motor_t`
的第二份持久运行状态。前者只描述控制反馈，后者只描述生命周期、故障和当前
命令。位置反馈由 `motor_t.tPositionFeedback` 保留一份最新结果；电流、电压和
duty 继续由收缩后的 Core 状态保留，查询时一次性复制。

依赖方向：

```text
foc_app ──调用──> motor
motor   ──调用──> foc_core + adc_ops + pwm_ops + position_ops
motor   ──可选调用──> position_ops.fnObserve(observer_input)
position provider ──可调用──> encoder / hall / SMO / NLFO / HFI+PLL
board port ──实现──> adc_ops + pwm_ops + 具体硬件 position provider
```

`motor` 是电机控制领域对象，目录建议放在 `foc/motor/`，不放入
`peripheral/driver/`。后者继续保存 AS5600、Hall GPIO、ABZ 定时器等具体
硬件驱动。

## 5. 数据结构草案

以下定义用于确认职责和所有权，字段名可在实现计划中按项目命名规则细化。

### 5.1 电机物理参数

```c
typedef enum {
    MOTOR_PARAM_VALID_RS   = (1UL << 0),
    MOTOR_PARAM_VALID_LD   = (1UL << 1),
    MOTOR_PARAM_VALID_LQ   = (1UL << 2),
    MOTOR_PARAM_VALID_FLUX = (1UL << 3),
} motor_param_valid_e;

typedef struct {
    foc_scalar_t qResistance;
    foc_scalar_t qInductanceD;
    foc_scalar_t qInductanceQ;
    foc_scalar_t qFlux;
    uint32_t wValidMask;
    uint8_t chPolePairs;
} motor_params_t;
```

采用 `Ld/Lq` 而不是单独的 `Ls`，原因是现有
`foc_identify_output_t` 已分别输出 `qInductanceD` 和 `qInductanceQ`，
`foc_feedforward` 与 MTPA 也分别消费 D/Q 电感。隐极电机使用
`Ld == Lq`；需要平均电感的 NLFO 在初始化时计算
`Ls = (Ld + Lq) / 2`。

`foc_identify` 分开识别 `Rs/Ld/Lq` 与 `Flux`，所以使用有效位表示部分结果。
极对数不能由当前识别流程获得，必须由产品配置提供。

第一阶段维持现有 FOC 归一化数值口径，不在此次结构重构中同时引入欧姆、
亨利和韦伯的 SI 基值换算。物理单位换算应单独设计，否则会把结构迁移和
算法标定风险混在一起。

### 5.2 控制初始化配置和运行状态

```c
typedef struct {
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    foc_scalar_t qHighFrequencyPeriod;
    uint16_t hwCalibrationTimeoutTicks;
} motor_control_cfg_t;
```

该结构只属于 `motor_cfg_t` 的初始化输入，不复制到 `motor_t`。初始化时将 PI
参数写入控制器实例；Motor 运行态只保留实际的积分器、历史误差和控制输出。
高频周期和校准超时只保留运行时必需的标量，不再保留完整配置结构。

当前 D/Q 电流环共用一组 PI 参数。第一阶段可以继续使用现有两个
`foc_pid_t`，但不得在 Motor 中再保存一份 `tCurrentPiParams`。若 RAM 优化成为
硬约束，下一步将 `foc_pid_t` 拆为“共享参数 + 每轴运行状态”，让 Id/Iq 共用一份
参数；这属于 PID 内部优化，不增加 Motor 层抽象。

`foc_core_state_t` 只保留电流环所需的运行数据：Id/Iq PI、αβ/dq 中间量、dq
电流、电压和最终 duty。以下字段不再属于 Core 状态：

- `tElectricalAngle`、`qElectricalSpeed`：来自位置反馈，Core 只读本拍输入；
- `qIu`、`qIv`、`qIw`：本拍输入，若波形需要则由可选诊断采样缓存保存；
- 位置源参数、极对数、零位和开环速度：由 Motor 参数或 provider 拥有。

### 5.3 最小位置源接口

所有 provider 对 `motor` 统一输出电角度和电速度。编码器/Hall provider 在
内部使用 `motor_params_t.chPolePairs`、方向和电气零位完成机械量换算；无感
provider 可直接输出电气量。

电流、电压、母线电压和采样周期不是“位置输入”，而是 SMO、NLFO 等无感
观察器的输入。通用位置接口因此拆成两个动作：所有 provider 都实现
`fnRead()`；只有无感 provider 实现可选的 `fnObserve()`。

```c
typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool bValid;
} motor_position_feedback_t;

typedef struct {
    foc_ab_t tCurrent;
    foc_ab_t tAppliedVoltage;
    foc_scalar_t qBusVoltage;
    foc_scalar_t qSamplePeriod;
} foc_observer_input_t;

typedef struct {
    foc_result_t (*fnInit)(void *pContext,
                           const motor_params_t *ptMotor,
                           foc_scalar_t qHighFrequencyPeriod);
    void (*fnReset)(void *pContext);
    int32_t (*fnSlowUpdate)(void *pContext);
    foc_result_t (*fnObserve)(
        void *pContext,
        const foc_observer_input_t *ptInput);
    foc_result_t (*fnRead)(
        void *pContext,
        motor_position_feedback_t *ptFeedback);
    foc_result_t (*fnCaptureElectricalZero)(void *pContext);
} motor_position_ops_t;

typedef struct {
    const motor_position_ops_t *ptOps;
    void *pContext;
} motor_position_t;
```

各类位置源的实现边界：

| provider | `fnSlowUpdate` | `fnObserve` | `fnRead` |
|---|---|---|---|
| AS5600 | I2C 读取并更新缓存 | `NULL` | 缓存读取与角度外推 |
| Hall | 更新 Hall 状态 | `NULL` | 输出扇区插值电角度 |
| ABZ 编码器 | 维护计数器与溢出 | `NULL` | 输出电角度和电速度 |
| SMO/NLFO | 可为空 | 消费电流、电压等观测量 | 输出估算结果 |
| HFI+SMO | 可为空 | provider 内部运行组合算法 | 输出最终结果 |

`motor.c` 不判断 provider 类型。它只在 `fnObserve != NULL` 时传入
`foc_observer_input_t`，随后统一调用 `fnRead()`。

接口刻意不包含以下内容：

- provider 数组和运行时注册表；
- 来源 ID、能力位和置信度；
- 通用资格判定、切换和角度混合；
- 通用双编码器一致性框架。

`fnCaptureElectricalZero` 是可选操作，只服务现有“转子对齐后记录电气零位”的
功能；无感 provider 可置为 `NULL`。Motor 只负责产生安全的对齐电流和调用
该操作，机械角读取、极对数换算和零位保存均由具体 provider 内部完成。
这不是能力查询或多源管理接口。

如果需要双编码器，新增 `dual_encoder_position_t`，它在内部比较两个编码器并
作为一个 `motor_position_t` 注册。如果需要 HFI+SMO，新增
`hfi_smo_position_t`，切换细节也封装在该 provider 内部。

### 5.4 Motor 配置与运行对象

```c
typedef enum {
    MOTOR_COMMAND_NONE = 0,
    MOTOR_COMMAND_START,
    MOTOR_COMMAND_STOP,
    MOTOR_COMMAND_CLEAR_FAULT,
    MOTOR_COMMAND_ADC_CALIBRATION,
} motor_command_e;

typedef struct {
    motor_command_e ePending;
} motor_command_sync_t;

typedef struct {
    motor_params_t tMotorParams;
    motor_control_cfg_t tControlCfg;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;
} motor_cfg_t;

typedef struct {
    motor_params_t tParams;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;

    foc_core_state_t tCore;
    foc_pid_t tSpeedPi;
    foc_adc_calib_t tAdcCalibration;
    motor_command_sync_t tCommandSync;
    foc_core_command_t tCommand;
    motor_lifecycle_t tLifecycle;
    motor_position_feedback_t tPositionFeedback;
} motor_t;

/* 仅作为 motor_GetFeedback() 的输出，不嵌入 motor_t。 */
typedef struct {
    motor_position_feedback_t tPosition;
    foc_dq_t                  tCurrent;
    foc_dq_t                  tVoltage;
    foc_duty_abc_t            tDuty;
} motor_feedback_t;

/* 仅作为 motor_GetStatus() 的输出，不嵌入 motor_t。 */
typedef struct {
    motor_lifecycle_t         tLifecycle;
    foc_core_command_t        tCommand;
} motor_status_t;
```

`motor_Init()` 只将运行所需的参数和接口复制到对象中；不会把完整的
`motor_control_cfg_t` 再保存一份。参数识别完成后可以安全更新 `motor_t.tParams`，
随后显式重置依赖该模型的 provider 和前馈模块。

ADC/PWM 回调在实施时统一增加首个 `void *pContext` 参数。STM32G431 当前实现
可以传 `NULL` 并继续访问唯一硬件实例；主机测试、其他 MCU 或未来双电机板则
可传入各自上下文。这样不会为了当前单例增加运行时管理器，但消除了接口对
全局变量的强依赖。

Motor 对外只提供完整操作 API，不暴露命令同步细节：

```c
motor_Init(&ptApp->tMotor, &tMotorCfg);
motor_Start(&ptApp->tMotor, &tCommand);
motor_Stop(&ptApp->tMotor);
motor_SetSpeedReference(&ptApp->tMotor, qSpeed);
motor_RequestAdcCalibration(&ptApp->tMotor);
motor_GetFeedback(&ptApp->tMotor, &tFeedback);
motor_GetStatus(&ptApp->tMotor, &tStatus);

motor_HighFrequencyStep(&ptApp->tMotor);  /* 20 kHz */
motor_ClockStep(&ptApp->tMotor);          /* 1 kHz */
motor_BackgroundStep(&ptApp->tMotor);     /* 前台调度 */
```

调用者只负责注册 `ops + context` 和调用上述 API，不调用
`motor_CommandSubmit()`、`motor_CommandConsume()`，也不直接改写 Motor
运行成员。

命令同步属于 `motor_t` 的内部子结构，不是独立 MODUS Class：

Mailbox 只负责在前台 API 与高频入口之间发布和消费命令，不理解生命周期、
PWM 或 FOC 业务。命令含义由 `motor.c` 解释；`motor_CommandSubmit()` 和
`motor_CommandConsume()` 为 `motor.c` 私有函数，并在内部使用中断保护。
当前只需要单槽“最新请求”语义，因此不单独创建 Mailbox Mdriver；如果未来
确实需要排队、优先级或多个 Class 复用，再把同一接口替换为独立模块。

### 5.5 状态唯一性约束

为保证重构后的 Motor 真正轻量化，以下数据只能有一个运行时拥有者：

| 数据 | 唯一拥有者 | 其他层的处理 |
|---|---|---|
| `Rs/Ld/Lq/Flux/PolePairs` | `motor_t.tParams` | provider 只读取或缓存派生量 |
| Id/Iq PI 与电流环工作量 | `foc_core_state_t` | `motor_t` 不再另存一份电流环 |
| 速度 PI 积分器 | `motor_t.tSpeedPi` | 配置只在初始化时传入 |
| 电气角度/电速度 | `motor_t.tPositionFeedback` | Core 只消费本拍输入 |
| 开环角度/开环速度 | 开环 provider context | 通用 Motor 不保存第二份 |
| 当前控制参考 | `motor_t.tCommand` | Mailbox 只保存待处理操作 |
| ADC 偏置与累加器 | `motor_t.tAdcCalibration` | Status 只复制快照 |
| provider 滤波、观测器状态 | provider context | Motor 不拆开管理 |
| 诊断计数和波形句柄 | `foc_app_t` | 不参与控制计算 |

`motor_feedback_t` 只作为 `motor_GetFeedback()` 的输出类型，不作为 `motor_t`
成员。它可以复制位置、电流、电压和 duty，供波形或上层控制读取，但不能反向
参与控制。`motor_status_t` 只作为 `motor_GetStatus()` 的输出类型，供 Shell、
故障诊断和产品状态机读取，不作为控制反馈输入。

`foc_core_state_t` 在实现中必须删除位置反馈和三相输入的持久副本，避免形成
“provider、Motor、Core”三份角度/速度或电流状态。若后续需要进一步节省 RAM，
再把 Id/Iq 两个 PID 的共享参数与每轴积分器拆开；该优化限制在 PID/Core 内部，
不新增 Motor 层对象。

### 5.6 大体实操示例

以下代码是拟定 API 的使用形态，用于说明对象怎样装配和调度；具体函数名在
实施阶段按项目命名规则落地。

以 STM32G431 + AS5600 为例，板级外设仍由 `peripheral_Init()` 初始化，产品层
只创建静态对象并把接口注册给 Motor：

```c
static as5600_position_t s_tAs5600Position;

typedef struct {
    modus_base_t tBase;
    motor_t tMotor;
    /* RingBuffer、Shell、波形和产品诊断成员。 */
} foc_app_t;

static const as5600_position_cfg_t s_tAs5600Cfg = {
    .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
    .hwInvalidTimeout = 100U,
    .bReverse = false,
};

static const motor_cfg_t s_tMotorCfg = {
    .tMotorParams = {
        .qResistance = FOC_SCALAR(...),
        .qInductanceD = FOC_SCALAR(...),
        .qInductanceQ = FOC_SCALAR(...),
        .qFlux = FOC_SCALAR(...),
        .chPolePairs = 7U,
    },
    .tControlCfg = {
        .tCurrentPiParams = { ... },
        .tSpeedPiParams = { ... },
        .qHighFrequencyPeriod = FOC_SCALAR(0.00005f),
        .hwCalibrationTimeoutTicks = 1000U,
    },
    .ptAdcOps = &g_tFocAdcOps,
    .pAdcContext = NULL,  /* 当前 STM32G431 为板级单例 */
    .ptPwmOps = &g_tFocPwmOps,
    .pPwmContext = NULL,
    .tPosition = {
        .ptOps = &g_tAs5600PositionOps,
        .pContext = &s_tAs5600Position,
    },
};

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;

    (void)wObjectCfgAddr;
    as5600_position_InitContext(&s_tAs5600Position, &s_tAs5600Cfg);
    return motor_Init(&ptThis->tMotor, &s_tMotorCfg);
}
```

这里 `as5600_position_InitContext()` 只复制参数和绑定已经初始化的 I2C 接口，
不发起 I2C 事务。实际调度保持三条固定路径：

```c
void foc_app_HighFrequencyISR(void)
{
    motor_HighFrequencyStep(&tFocApp.tMotor);
}

int foc_app_Run(foc_app_t *ptThis)
{
    motor_BackgroundStep(&ptThis->tMotor);  /* AS5600 I2C 更新发生在这里 */
    return MODUS_SUCCESS;
}
```

`foc_app_Clock()` 只在严格的 1 kHz 时基调用
`motor_ClockStep(&ptThis->tMotor)`，执行速度 PI；`foc_app_Run()` 只调用
`motor_BackgroundStep()`，执行 AS5600 I2C 和其他非实时服务。BackgroundStep
不消费 Motor 命令，也不修改运行状态。

第一次 `motor_HighFrequencyStep()` 自动开始 ADC 零偏校准；完成后 Motor 进入
`IDLE`，收到 Start 命令才开启功率输出。AS5600 的 `fnObserve` 为 `NULL`，
高频只调用 `fnRead()` 读取缓存和外推结果。

如果改用 SMO，只替换注册对象：

```c
.tPosition = {
    .ptOps = &g_tSmoPositionOps,
    .pContext = &s_tSmoPosition,
},
```

SMO 的 `fnObserve()` 在高频拍消费电流、上一拍已施加电压、母线电压和周期，
`fnRead()` 输出估算角度。`foc_app`、ISR 和 `motor.c` 的控制流程不需要修改。

## 6. 生命周期与首次调度初始化

```text
UNINITIALIZED
    │ motor_Init：校验并注册接口，立即关闭 PWM
    ▼
INITIALIZING
    │ 首次高频调度：开始 ADC 偏置采样，复位位置源和控制器
    ▼
CALIBRATING
    │ 每个 ADC 中断累计一个偏置样本
    ├── 完成 ───────────────> IDLE
    └── 超时/非法 ──────────> FAULT

IDLE ── motor_Start ────────> RUNNING
RUNNING ── motor_Stop ──────> IDLE
RUNNING ── 任一关键错误 ───> FAULT
FAULT ── motor_ClearFault ──> INITIALIZING
```

规则：

1. `motor_Init()` 只做参数复制、接口注册、合法性检查和急停，不做阻塞操作。
2. `motor_Init()` 调用位置源 `fnInit` 时只允许建立软件状态和绑定配置；板级
   外设应已由 `foc_port` 初始化，I2C/SPI 事务不能在这里执行。
3. ADC 偏置在第一次高频调度中调用现有 `fnCalibrationBegin()` 启动，并在
   后续高频拍通过现有 `fnCalibrationStep()` 逐步完成。
4. PWM 在 `INITIALIZING`、`CALIBRATING`、`IDLE` 和 `FAULT` 中始终关闭；
   STM32G431 只开启 TIM1 CH4 ADC 触发，不开启 CH1~CH3 功率输出。
5. `motor_Start()` 只允许从已完成初始化的 `IDLE` 进入运行态，不再每次启动
   都自动重新校准 ADC。
6. `motor_ClearFault()` 回到 `INITIALIZING`，统一重新建立偏置和算法初值，
   避免区分大量故障恢复分支。
7. 位置源 `fnInit`、`fnReset`、`fnObserve` 和 `fnRead` 必须无阻塞；I2C/SPI
   更新只能放在 `fnSlowUpdate`，由 `motor_BackgroundStep()` 在非 ISR 调度中调用。

## 7. 高频和慢速数据流

### 7.1 高频路径

```text
ADC1_2_IRQHandler
  → motor_HighFrequencyStep(&tFocApp.tMotor)
      → 生命周期/命令处理
      → adc_ops.fnCurrentSample()
      → 构造 foc_observer_input_t
      → position_ops.fnObserve(...)  // 仅无感 provider，NULL 时跳过
      → position_ops.fnRead(...)
      → foc_core_step(current, electrical_angle, command)
      → pwm_ops.fnDutyCommit()
```

只有无感观察器使用“上一拍已施加的 αβ 电压”作为观测输入，避免在同一拍中
使用尚未提交的输出造成时序含义不明确。编码器和 Hall 不接收这些数据。

### 7.2 慢速路径

```text
MODUS Clock
  → motor_ClockStep(&tFocApp.tMotor)
      → speed PI                      // 严格 1 kHz，仅 SPEED 模式

MODUS Run / PT
  → motor_BackgroundStep(&tFocApp.tMotor)
      → position_ops.fnSlowUpdate()   // AS5600 I2C、ABZ 维护等
  → foc_app diagnostics / waveform / shell
```

`foc_app` 不再直接读取 AS5600、不再换算极对数、不再持有电流环、速度环或
ADC 偏置状态，也不再消费 Motor 内部命令。

## 8. 参数识别接入方式

当前 `foc_identify` 输出与 `motor_params_t` 的映射为：

```text
qResistance  ──> motor_params_t.qResistance
qInductanceD ──> motor_params_t.qInductanceD
qInductanceQ ──> motor_params_t.qInductanceQ
qFlux        ──> motor_params_t.qFlux
```

识别模块不直接成为 `motor_t` 的永久成员。它保持为可选实验控制器，由上层在
电机停止且安全条件满足时临时驱动。Motor 也不能依赖实验模块的数据类型，
因此只提供通用参数应用 API：

```c
foc_result_t motor_ApplyParams(
    motor_t *ptMotor,
    const motor_params_t *ptParams);
```

识别适配层负责把 `foc_identify_output_t` 转成 `motor_params_t`；从 Flash、产品
配置或其他参数识别算法得到的结果也走同一个入口。该 API 只允许在 `IDLE`
状态调用，按 `wValidMask` 更新字段后重置依赖物理模型的模块。参数识别的上电
激励、安全监护和保存到 Flash 不纳入第一阶段结构迁移。

## 9. 文件结构

计划新增：

```text
foc/motor/motor.h              motor_t、配置、状态和公开 API
foc/motor/motor.c              生命周期、高频步和慢速步
foc/motor/motor_params.h       电机物理参数及有效位
foc/motor/motor_position.h     最小位置源接口
foc/observer/foc_observer.h    无感观察器输入结构
tests/foc/test_motor.c         Motor 对象和生命周期主机测试
```

计划修改：

```text
foc/app/foc_app.h/.c           缩减为 MODUS/Shell/波形适配，组合 motor_t
foc/hal/foc_port.h             ADC/PWM ops 增加 context，移除编码器专属参数
foc/middleware/foc_core.h/.c   收缩 Core 状态，移除位置和三相输入持久副本
peripheral/stm32g431/foc_port.c 注册板级 ADC/PWM 和 AS5600 provider
peripheral/driver/as5600.h/.c  实现 motor_position_ops_t
foc/observer/foc_encoder.h/.c  参数只保留编码器滤波和样本超时
foc/foc.mk                     加入 motor 源文件
tests/foc/Makefile             增加 float/fixed Motor 测试目标
```

高级观察器文件先不加入固件构建。待 `motor_position.h` 稳定后，再分别迁移
SMO、NLFO、Hall 和 HFI 组合 provider，每次迁移独立测试。

## 10. 分阶段实施顺序

### 阶段 1：冻结现有行为

- 补齐 `foc_app` 当前生命周期、高频调用顺序和故障急停测试。
- 记录 float/fixed 主机测试和 STM32G431 固件构建基线。

### 阶段 2：建立 Motor 类型和最小位置接口

- 新增 `motor_params_t`、`motor_position_t`、`motor_cfg_t` 和 `motor_t`。
- `motor_cfg_t` 只作为初始化输入；禁止在 `motor_t` 中重复保存完整的控制配置。
- 使用 stub ADC/PWM/position 接口验证初始化和首次调度校准。
- 此阶段不接入 `foc_app`，保证新对象可以独立测试。

### 阶段 3：迁移高频闭环和生命周期

- 将命令邮箱、ADC 校准、故障处理、`foc_core_step()` 和 PWM 提交移入
  `motor.c`。
- 收缩 `foc_core_state_t`，删除角度、速度和三相输入的持久副本；保留电流环
  所需的 PI 状态、控制中间量和输出。
- 确认速度 PI、当前生效命令、位置反馈和 provider 私有状态各自只有一个运行时
  拥有者。
- 保持当前中性 duty 后使能 PWM、故障第一动作急停等安全不变量。

### 阶段 4：迁移位置源

- 将 `tEncoderParams.chPolePairs` 移入 `motor_params_t`。
- 将高频周期和校准超时移入 `motor_control_cfg_t`，但不把整个配置结构复制到
  `motor_t`。
- AS5600 provider 只保留滤波、超时、方向和零位等自身参数。
- `motor` 只消费统一的电角度、电速度和 `bValid`。
- 电流、电压、母线电压和周期只经可选 `fnObserve()` 传给无感 provider，
  编码器/Hall provider 不接收这些输入。
- 将现有编码器零位对齐流程迁移为 Motor 的可选命令；对齐完成后只调用活动
  provider 的 `fnCaptureElectricalZero()`，不判断 provider 类型。

### 阶段 5：缩减 foc_app

- `foc_app_t` 组合一个 `motor_t`。
- ISR、Clock、Run、Shell 和 waveform 全部通过 Motor API 访问。
- 删除 `foc_app` 中重复的 Core、PI、ADC、位置换算和生命周期成员。

### 阶段 6：通用参数入口与识别映射

- 添加 `motor_ApplyParams()` 及有效位更新测试。
- 在实验模块侧添加 `foc_identify_output_t -> motor_params_t` 纯数据映射。
- 保持实验识别功能默认关闭，不在本阶段接入自动激励流程。
- 更新 FOC README、架构文档和 Archify 当前架构图。

## 11. 测试边界

必须覆盖：

- 配置为空、ops 缺失、极对数为零时初始化失败并保持 PWM 关闭。
- 第一次高频调度只开始偏置校准，不执行 FOC、不提交工作 duty。
- 校准完成前不能 Start；完成后进入 IDLE。
- RUNNING 调用顺序为采样、位置、Core、DutyCommit。
- 位置无效、采样失败、Core 失败或 DutyCommit 失败时第一动作急停。
- Stop 立即关闭 PWM。
- ClearFault 回到 INITIALIZING 并重新校准。
- AS5600 provider 的慢速 I2C 和快速缓存读取严格分离。
- 编码器、Hall 和无感 stub 都可通过同一个 `motor_position_t` 接口运行。
- `fnObserve == NULL` 时高频路径直接读取位置；非空时先执行观察再读取结果。
- 支持零位捕获的 provider 能完成现有编码器对齐；不支持的 provider 明确
  由 `motor_RequestPositionCalibration()` 返回现有的 `FOC_RESULT_DISABLED`，
  不增加能力表。
- `motor_ApplyParams()` 可接收配置、Flash 或识别映射结果，并且只在 IDLE
  按有效位更新参数。
- 同一套 ADC/PWM ops 可以配合两个不同 context 通过主机对象测试，证明接口
  不依赖隐藏全局状态。
- 结构审查确认 `motor_t` 不含完整的 `motor_control_cfg_t`，Core 不含位置和
  三相输入的持久副本，`motor_feedback_t` 只由状态查询临时生成。
- `motor_GetFeedback()` 只返回位置、电流、电压和 duty；`motor_GetStatus()` 只
  返回生命周期、故障和当前命令，两个输出结构不能互相替代。
- float 与 Q16.15 后端保持同一套行为测试。

## 12. 可适配性保证

“不提前内置能力”不等于封闭扩展点。基础框架通过稳定接口和对象组合保证
适配性，新增产品能力时优先新增实现，不修改 `motor.c` 主流程。

| 变化需求 | 扩展位置 | 是否修改 Motor 主流程 |
|---|---|---|
| 更换 MCU、ADC 拓扑或 PWM 外设 | 新 ADC/PWM ops + context | 否 |
| 更换 AS5600、Hall、ABZ 等有感位置源 | 新 position provider | 否 |
| 增加 SMO、NLFO、HFI 等无感算法 | 实现 `fnObserve` 的 provider | 否 |
| 双编码器或 HFI+SMO 组合 | 一个复合 position provider | 否 |
| 参数来自产品配置、Flash 或新识别算法 | 转成 `motor_params_t` 后调用 `motor_ApplyParams` | 否 |
| float、Q16.15 或后续数值后端 | 复用现有 `foc_scalar_t` 数值抽象 | 否 |
| 增加新的控制模式或环路 | 独立 controller，再由 Motor 明确编排 | 需要评审 |

基础接口必须保持以下约束：

1. `motor.h/.c` 不包含 AS5600、Hall、SMO、具体 MCU 或 `foc_identify` 头文件。
2. `motor.c` 不出现按 provider 类型分支的枚举或 `switch`；差异由 ops 实现。
3. ADC、PWM 和 position 都使用 `ops + context`，允许同一份实现绑定不同实例。
4. 高频接口只处理确定时长的计算和缓存读取；总线访问固定留在慢速接口。
5. 只有运行时必需的配置值复制进 `motor_t`；初始化专用配置不作为第二份状态
   保存。具体 provider 私有状态留在各自 context，所有权清晰且没有动态分配。
6. `motor_feedback_t` 和 `motor_status_t` 只作为查询输出，不作为 Motor 内部
   持久快照；Core、position provider 和诊断之间不重复拥有同一运行值。
7. 可选能力使用单个可选回调或复合 provider，不增加通用注册表、能力位和
   多源状态机。
8. Motor 参数只通过 `motor_ApplyParams()` 更新，参数来源不进入 Motor 领域层。

实施验收时至少使用 AS5600、一个无感 stub、两组硬件 context 和两种数值后端
编译同一套 Motor 测试。只有同时通过，才认为接口具备预期适配性。

## 13. 本草案明确不做的内容

- 通用多位置源注册表。
- 运行时来源选择、资格判定和无扰切换。
- 通用双编码器投票或一致性管理器。
- 自动参数识别启动、Flash 持久化和 SI 单位体系改造。
- MTPA、弱磁、位置环、扭矩环和复杂前馈的同时接入。
- 多电机全局管理器。

这些能力不提前放进基础框架，但对应扩展点必须满足第 12 节契约。以后增加
具体能力时，应优先新增 provider、controller 或参数适配层，不破坏已经稳定的
Motor 生命周期和高频数据流。

## 14. 评审重点

本轮请重点判断以下设计取向是否符合预期：

1. `motor_t` 是 FOC 领域对象，位于 `foc/motor/`，而不是具体硬件 driver。
2. 初始化时只注册一个位置 provider，组合与切换由具体 provider 自己负责。
3. ADC 偏置在首次高频调度自动完成，正常 Stop/Start 不重复校准。
4. 电机模型采用 `Rs/Ld/Lq/Flux/PolePairs`，与现有参数识别输出对齐。
5. `foc_app` 最终只保留 MODUS、Shell、波形和产品适配职责。
6. ADC/PWM/position 都使用 `ops + context`，Motor 不依赖具体硬件或算法类型。
7. 参数入口采用 `motor_ApplyParams()`，识别、Flash 和产品配置在外部适配。
8. Motor 只保留一份运行状态，配置、位置反馈、Core 输出、诊断和命令同步不
   发生重复拥有。
