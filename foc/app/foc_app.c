/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS Class for the single-motor FOC application.
 * @author  Codex
 * @date    2026-08-31
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_app.h"
#include "foc_config.h"
#include "foc_port.h"
#include "foc_core.h"
#include "foc_encoder.h"
#include "foc_pid.h"
#include "mdebug/util_debug.h"

#if defined(MODUS_ENABLE) && MODUS_ENABLE
#include "perfc_task_pt.h"
#endif

#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
#include "mdebug/mwaveform.h"
#endif

#if defined(MODUS_ENABLE) && MODUS_ENABLE
#include "mdebug/mshell.h"
#endif

#define FOC_APP_CALIBRATION_MAX_TICKS  2000U
#define FOC_APP_HF_PERIOD_S            0.00005f
#define FOC_APP_CURRENT_ALIGN_TICKS    20000U
#define FOC_APP_OPEN_LOOP_ACCEL        0.25f
/* 编码器零位标定对齐保持拍数：1500 ms @ 20 kHz。对齐电流 Id 定电流、
 * Iq=0，力矩 T∝Id×ψ 不依赖 R/L/母线电压散布，比 Vd 电压对齐更确定。 */
#define FOC_APP_ENCODER_CAL_TICKS      30000U
#define FOC_APP_ENC_CAL_ID_ALIGN       0.10f

static void foc_app_CurrentStartupStep(foc_app_t *ptThis);

#if defined(MODUS_ENABLE) && MODUS_ENABLE
static modus_base_t s_tFocAppBase;
static int foc_app_Clock(uintptr_t wObjectAddr);
static int foc_app_Run(uintptr_t wObjectAddr);

static modus_base_cfg_t s_tFocAppBaseCfg = {
    .wId = FOC_APP,
    .wParent = 0U,
    .pchRingBuffer = NULL,
    .hwRingSize = 0U,
    .FcnInterface = {
        .Clock = foc_app_Clock,
        .Run = foc_app_Run,
    },
};

static uint8_t s_chFocAppRingBuffer[128];

// foc_app_t tFocApp;
MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tCurrentPiParams = {
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.55f),
        .qOutputMaximum = FOC_SCALAR(0.55f),
        .qIntegratorMinimum = FOC_SCALAR(-0.50f),
        .qIntegratorMaximum = FOC_SCALAR(0.50f),
    },
    .tSpeedPiParams = {
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .tKdOverTs = {0, FOC_ZERO},
        .qOutputMinimum = FOC_SCALAR(-0.10f),
        .qOutputMaximum = FOC_SCALAR(0.10f),
        .qIntegratorMinimum = FOC_SCALAR(-0.10f),
        .qIntegratorMaximum = FOC_SCALAR(0.10f),
    },
    .tEncoderParams = {
        .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
        .hwInvalidTimeout = 100U,
        .chPolePairs = 7U,
        .qHighFrequencyPeriod = FOC_SCALAR(FOC_APP_HF_PERIOD_S),
    },
    .tElectricalZero = {0U},
    .ptPwmOps = &g_tFocPwmOps,
    .ptAdcOps = &g_tFocAdcOps,
    .tSensor = { .ptOps = NULL, .pPriv = NULL },
    .chFeedbackSlot = 0U,
    .pchRingBuffer = s_chFocAppRingBuffer,
    .hwRingSize = sizeof(s_chFocAppRingBuffer),
    .bDirectionInverted = false
)
#endif

/**
 * @brief Validate the configuration contract before touching the object.
 * @param ptCfg Configuration supplied by the object owner.
 * @return true when all mandatory dependencies and limits are valid.
 */
static bool foc_app_ConfigValid(const foc_app_cfg_t *ptCfg)
{
    bool bValid = true;

    if (ptCfg == NULL || ptCfg->pchRingBuffer == NULL ||
        ptCfg->hwRingSize == 0U || ptCfg->ptPwmOps == NULL ||
        ptCfg->ptAdcOps == NULL) {
        bValid = false;
    }
    if (ptCfg != NULL &&
        (ptCfg->tEncoderParams.chPolePairs == 0U ||
         ptCfg->tEncoderParams.hwInvalidTimeout == 0U ||
         ptCfg->tEncoderParams.qHighFrequencyPeriod <= FOC_ZERO ||
         ptCfg->tEncoderParams.qSpeedFilterAlpha < FOC_ZERO ||
         ptCfg->tEncoderParams.qSpeedFilterAlpha > FOC_ONE)) {
        bValid = false;
    }
    return bValid;
}

static foc_result_t foc_app_MotorPositionInit(
    void *pContext,
    const motor_params_t *ptMotor,
    foc_scalar_t qHighFrequencyPeriod)
{
    if (pContext == NULL || ptMotor == NULL ||
        ptMotor->chPolePairs == 0U || qHighFrequencyPeriod <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

static void foc_app_MotorPositionReset(void *pContext)
{
    (void)pContext;
}

static int32_t foc_app_MotorPositionSlowUpdate(void *pContext)
{
    foc_app_t *ptThis = (foc_app_t *)pContext;
    int32_t nResult = 0;
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    uint32_t wNow = 0U;
    uint32_t wIntervalMs = 1U;

    if (ptThis == NULL) {
        return -1;
    }
    /* Motor 路径下的慢速轮询必须与旧 EncoderPoll 相同：限速 1 kHz，
       失败退避 100 ms，避免随 Run 节奏反复访问 I2C 总线。 */
    wNow = (uint32_t)get_system_ms();
    if (ptThis->tDiagnostics.hwConsecutivePollFails > 0U) {
        wIntervalMs = 100U;
    }
    if ((uint32_t)(wNow - ptThis->tDiagnostics.wLastPollMs) <
        wIntervalMs) {
        return 0;
    }
    ptThis->tDiagnostics.wLastPollMs = wNow;
#else
    if (ptThis == NULL) {
        return -1;
    }
#endif
    if (ptThis->tSensor.ptOps == NULL ||
        ptThis->tSensor.ptOps->fnUpdate == NULL) {
        return 0;
    }
    nResult = ptThis->tSensor.ptOps->fnUpdate(ptThis->tSensor.pPriv);
    if (nResult < 0) {
#if defined(MODUS_ENABLE) && MODUS_ENABLE
        if (ptThis->tDiagnostics.hwConsecutivePollFails < UINT16_MAX) {
            ptThis->tDiagnostics.hwConsecutivePollFails++;
        }
#endif
        return nResult;
    }
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    ptThis->tDiagnostics.hwConsecutivePollFails = 0U;
#if defined(FOC_NUMERIC_FLOAT)
    {
        foc_angle_t tMechanicalAngle = {0U};
        foc_scalar_t qMechanicalSpeed = FOC_ZERO;
        bool bValid = false;

        /* EncMech 波形只读缓存的机械角快照，不发起总线访问。 */
        if (ptThis->tSensor.ptOps->fnRead != NULL &&
            ptThis->tSensor.ptOps->fnRead(
                ptThis->tSensor.pPriv, &tMechanicalAngle,
                &qMechanicalSpeed, &bValid) == FOC_RESULT_OK && bValid) {
            ptThis->tDiagnostics.fEncoderMechanicalTurns =
                foc_angle_to_turns(tMechanicalAngle);
        }
    }
#endif
#endif
    return 0;
}

static foc_result_t foc_app_MotorPositionRead(
    void *pContext,
    motor_position_feedback_t *ptFeedback)
{
    foc_app_t *ptThis = (foc_app_t *)pContext;
    foc_angle_t tMechanicalAngle = {0U};
    foc_scalar_t qMechanicalSpeed = FOC_ZERO;
    uint64_t llElectrical = 0U;
    uint32_t wElectricalAngle = 0U;
    bool bValid = false;

    if (ptThis == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->tPosition.eAngleSource == FOC_APP_ANGLE_OPEN_LOOP) {
        foc_app_CurrentStartupStep(ptThis);
        ptThis->tPosition.tOpenLoopAngle = foc_angle_add_scalar(
            ptThis->tPosition.tOpenLoopAngle,
            foc_mul_pu(ptThis->tPosition.qOpenLoopSpeed,
                       ptThis->tMotor.qHighFrequencyPeriod));
        ptFeedback->tElectricalAngle = ptThis->tPosition.tOpenLoopAngle;
        ptFeedback->qElectricalSpeed = ptThis->tPosition.qOpenLoopSpeed;
        ptFeedback->bValid = true;
        return FOC_RESULT_OK;
    }
    if (ptThis->tSensor.ptOps == NULL ||
        ptThis->tSensor.ptOps->fnRead == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (ptThis->tSensor.ptOps->fnRead(
            ptThis->tSensor.pPriv, &tMechanicalAngle,
            &qMechanicalSpeed, &bValid) != FOC_RESULT_OK || !bValid) {
        ptFeedback->bValid = false;
        return FOC_RESULT_OK;
    }
    llElectrical = (uint64_t)tMechanicalAngle.wBam32 *
                   (uint64_t)ptThis->tPosition.chPolePairs;
    wElectricalAngle = (uint32_t)llElectrical;
    if (ptThis->tPosition.bDirectionInverted) {
        wElectricalAngle = 0U - wElectricalAngle;
    }
    ptFeedback->tElectricalAngle = foc_angle_add(
        (foc_angle_t){wElectricalAngle},
        ptThis->tPosition.tElectricalZero);
    ptFeedback->qElectricalSpeed = foc_mul_wide(
        qMechanicalSpeed,
        FOC_SCALAR((float)ptThis->tPosition.chPolePairs));
    ptFeedback->bValid = true;
    return FOC_RESULT_OK;
}

/**
 * @brief Capture the encoder position as the electrical zero.
 * @param pContext App object serving as the position provider context.
 * @return FOC_RESULT_OK after storing the zero; SAFETY when the
 *         encoder sample is not valid, DISABLED when it cannot read.
 * @note The rotor is held by the alignment current when Motor calls
 *       this operation; only the cached sensor sample is consumed.
 */
static foc_result_t foc_app_MotorPositionCaptureElectricalZero(
    void *pContext)
{
    foc_app_t *ptThis = (foc_app_t *)pContext;
    foc_angle_t tMechanicalAngle = {0U};
    foc_scalar_t qMechanicalSpeed = FOC_ZERO;
    bool bValid = false;
    float fMechanicalTurns = 0.0f;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->tSensor.ptOps == NULL ||
        ptThis->tSensor.ptOps->fnRead == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (ptThis->tSensor.ptOps->fnRead(
            ptThis->tSensor.pPriv, &tMechanicalAngle,
            &qMechanicalSpeed, &bValid) != FOC_RESULT_OK || !bValid) {
        return FOC_RESULT_SAFETY;
    }
    fMechanicalTurns = foc_angle_to_turns(tMechanicalAngle);
    ptThis->tPosition.tElectricalZero = foc_angle_from_turns(
        -fMechanicalTurns * (float)ptThis->tPosition.chPolePairs);
    ptThis->tPosition.bEncoderCalibrated = true;
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    MLOGF(I, "encoder cal: mech=%.4f turn offset=%.4f turn\r\n",
          (double)fMechanicalTurns,
          (double)foc_angle_to_turns(
              ptThis->tPosition.tElectricalZero));
#endif
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tFocAppMotorPositionOps = {
    .fnInit = foc_app_MotorPositionInit,
    .fnReset = foc_app_MotorPositionReset,
    .fnSlowUpdate = foc_app_MotorPositionSlowUpdate,
    .fnObserve = NULL,
    .fnRead = foc_app_MotorPositionRead,
    .fnCaptureElectricalZero =
        foc_app_MotorPositionCaptureElectricalZero,
};

/**
 * @brief Keep the float electrical-angle snapshot used by the waveform.
 * @param ptThis FOC application object.
 * @return None.
 * @note 状态/电流/电压均直接读 Motor，此处只维护波形需要的角度换算。
 */
static void foc_app_SyncMotorView(foc_app_t *ptThis)
{
    if (ptThis == NULL) {
        return;
    }
#if defined(FOC_NUMERIC_FLOAT)
    ptThis->tDiagnostics.fElectricalAngleTurns =
        foc_angle_to_turns(
            ptThis->tMotor.tPositionFeedback.tElectricalAngle);
#endif
}

/**
 * @brief Test whether the injected encoder can provide an electrical angle.
 * @param ptThis FOC application object.
 * @return true when the encoder is enabled and calibrated.
 */
static bool foc_app_EncoderReady(const foc_app_t *ptThis)
{
    bool bReady = false;

    if (ptThis != NULL) {
        bReady = ptThis->tPosition.bEncoderEnabled &&
                 ptThis->tPosition.bEncoderCalibrated &&
                 ptThis->tSensor.ptOps != NULL &&
                 ptThis->tPosition.chPolePairs > 0U;
    }
    return bReady;
}

/**
 * @brief Validate a requested control mode.
 * @param eMode Control mode to check.
 * @return true for a supported mode.
 */
static bool foc_app_ModeValid(foc_control_mode_e eMode)
{
    return eMode <= FOC_MODE_SPEED;
}

/**
 * @brief Apply the safe current-mode handoff and open-loop ramp.
 * @param ptThis FOC application object.
 * @return None.
 */
static void foc_app_CurrentStartupStep(foc_app_t *ptThis)
{
    foc_scalar_t qRampStep = FOC_ZERO;
    foc_core_command_t *ptCommand = NULL;

    if (ptThis == NULL) {
        return;
    }
    /* 对齐/斜坡只作用于 Motor 当前命令（产品路径恒为 Motor）。 */
    ptCommand = &ptThis->tMotor.tCommand;
    if (ptThis->tPosition.bCurrentStartupAlign) {
        ptThis->tPosition.qOpenLoopSpeed = FOC_ZERO;
        ptCommand->eMode = FOC_MODE_VOLTAGE;
        ptCommand->tVoltageReference.qD = FOC_SCALAR(0.04f);
        ptCommand->tVoltageReference.qQ = FOC_ZERO;
        if (ptThis->tPosition.wCurrentAlignTicks < UINT32_MAX) {
            ptThis->tPosition.wCurrentAlignTicks++;
        }
        if (ptThis->tPosition.wCurrentAlignTicks >=
            FOC_APP_CURRENT_ALIGN_TICKS) {
            if (ptThis->tPosition.bEncoderCalibrated) {
                ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_ENCODER;
            }
            ptThis->tPosition.bCurrentStartupAlign = false;
            ptCommand->eMode = FOC_MODE_CURRENT;
        }
    } else if (ptCommand->eMode == FOC_MODE_CURRENT) {
        qRampStep = foc_mul_pu(FOC_SCALAR(FOC_APP_OPEN_LOOP_ACCEL),
                               FOC_SCALAR(FOC_APP_HF_PERIOD_S));
        if (ptThis->tPosition.qOpenLoopTargetSpeed >
            ptThis->tPosition.qOpenLoopSpeed) {
            ptThis->tPosition.qOpenLoopSpeed = foc_add_sat(
                ptThis->tPosition.qOpenLoopSpeed, qRampStep);
            if (ptThis->tPosition.qOpenLoopSpeed >
                ptThis->tPosition.qOpenLoopTargetSpeed) {
                ptThis->tPosition.qOpenLoopSpeed =
                    ptThis->tPosition.qOpenLoopTargetSpeed;
            }
        } else if (ptThis->tPosition.qOpenLoopTargetSpeed <
                   ptThis->tPosition.qOpenLoopSpeed) {
            ptThis->tPosition.qOpenLoopSpeed = foc_sub_sat(
                ptThis->tPosition.qOpenLoopSpeed, qRampStep);
            if (ptThis->tPosition.qOpenLoopSpeed <
                ptThis->tPosition.qOpenLoopTargetSpeed) {
                ptThis->tPosition.qOpenLoopSpeed =
                    ptThis->tPosition.qOpenLoopTargetSpeed;
            }
        } else {
            /* The requested open-loop speed has been reached. */
        }
    } else {
        /* Voltage and speed modes do not use this handoff. */
    }
}

/**
 * @brief Execute the object-bound hard real-time FOC step.
 * @param ptThis FOC application object.
 * @return None.
 * @note This path contains no logging, I2C, PT or blocking operation.
 */
void foc_app_HighFrequencyStep(foc_app_t *ptThis)
{
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    int64_t lIsrCycles = 0;
    uint32_t wCycles = 0U;

    start_cycle_counter();
#endif

    if (ptThis == NULL) {
        return;
    }
    motor_HighFrequencyStep(&ptThis->tMotor);
    foc_app_SyncMotorView(ptThis);
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    mwaveform.Step();
#endif
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    lIsrCycles = stop_cycle_counter();
    if (lIsrCycles > 0) {
        wCycles = (lIsrCycles > (int64_t)UINT32_MAX)
            ? UINT32_MAX : (uint32_t)lIsrCycles;
        if (wCycles > ptThis->tDiagnostics.wIsrMaxCycles) {
            ptThis->tDiagnostics.wIsrMaxCycles = wCycles;
        }
        if (ptThis->tDiagnostics.wIsrSamples < UINT32_MAX) {
            ptThis->tDiagnostics.wIsrSamples++;
        }
    }
#endif
}

#if defined(MODUS_ENABLE) && MODUS_ENABLE
/**
 * @brief Keep the high-frequency timing snapshot outside the ISR.
 * @param ptThis FOC application object.
 * @param bReset Clear the accumulated maximum and sample count.
 * @return None.
 */
static void foc_app_IsrTimingPrint(foc_app_t *ptThis, bool bReset)
{
    uint32_t wMaxCycles = 0U;
    uint32_t wSamples = 0U;
    uintptr_t wState = 0U;
#if FOC_HF_DEADLINE_CYCLES > 0U
    uint32_t wLoadPermille = 0U;
    uint32_t wHeadroomCycles = 0U;
#endif

    if (ptThis == NULL) {
        return;
    }
    wState = perfc_port_disable_global_interrupt();
    wMaxCycles = ptThis->tDiagnostics.wIsrMaxCycles;
    wSamples = ptThis->tDiagnostics.wIsrSamples;
    if (bReset) {
        ptThis->tDiagnostics.wIsrMaxCycles = 0U;
        ptThis->tDiagnostics.wIsrSamples = 0U;
    }
    perfc_port_resume_global_interrupt(wState);
#if FOC_HF_DEADLINE_CYCLES > 0U
    wLoadPermille = (uint32_t)(
        ((uint64_t)wMaxCycles * 1000U) /
        (uint64_t)FOC_HF_DEADLINE_CYCLES);
    wHeadroomCycles = wMaxCycles < FOC_HF_DEADLINE_CYCLES
        ? FOC_HF_DEADLINE_CYCLES - wMaxCycles : 0U;
    MLOGF(T, "[FOC] HF ISR: max=%lu us (%lu cyc) load=%lu.%lu%% "
          "headroom=%lu cyc n=%lu\r\n",
          (unsigned long)perfc_convert_ticks_to_us(
              (int64_t)wMaxCycles),
          (unsigned long)wMaxCycles,
          (unsigned long)(wLoadPermille / 10U),
          (unsigned long)(wLoadPermille % 10U),
          (unsigned long)wHeadroomCycles,
          (unsigned long)wSamples);
#else
    MLOGF(T, "[FOC] HF ISR: max=%lu us (%lu cyc) n=%lu\r\n",
          (unsigned long)perfc_convert_ticks_to_us(
              (int64_t)wMaxCycles),
          (unsigned long)wMaxCycles,
          (unsigned long)wSamples);
#endif
}

/**
 * @brief Report ISR timing once per second from the App object.
 * @param ptThis FOC application object.
 * @return None.
 */
static void foc_app_IsrTimingReport(foc_app_t *ptThis)
{
    uint32_t wNow = 0U;

    if (ptThis == NULL) {
        return;
    }
    wNow = (uint32_t)get_system_ms();
    if ((uint32_t)(wNow - ptThis->tDiagnostics.wLastReportMs) >= 1000U) {
        ptThis->tDiagnostics.wLastReportMs = wNow;
        foc_app_IsrTimingPrint(ptThis, true);
    }
}

#endif

#if defined(MODUS_ENABLE) && MODUS_ENABLE

/**
 * @brief MODUS Clock callback for the 1 kHz speed loop.
 * @param wObjectAddr Address of the owning App object.
 * @return MODUS_SUCCESS or an error code.
 */
static int foc_app_Clock(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    motor_ClockStep(&ptThis->tMotor);
    foc_app_SyncMotorView(ptThis);
    return MODUS_SUCCESS;
}

/**
 * @brief MODUS Run callback that dispatches the App PT.
 * @param wObjectAddr Address of the owning App object.
 * @return MODUS_SUCCESS or an error code.
 */
static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->tDiagnostics.chRunPt)
    while (1) {
        motor_BackgroundStep(&ptThis->tMotor);
        foc_app_SyncMotorView(ptThis);
        foc_app_IsrTimingReport(ptThis);
        PERFC_PT_YIELD(MODUS_SUCCESS);
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}
#endif

/**
 * @brief Add one float waveform variable and report registration status.
 * @param pchName Waveform channel name.
 * @param fScale Display scale.
 * @param pvValue Address of the sampled value.
 * @return true when the channel was registered.
 */
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
typedef struct {
    const char *pchName;
    float fScale;
    void *pvValue;
} foc_app_wave_channel_t;

/**
 * @brief Register the App-owned real-time waveform variables.
 * @param ptThis FOC application object.
 * @return None.
 */
static void foc_app_WaveformInit(foc_app_t *ptThis)
{
    const foc_app_wave_channel_t atChannels[] = {
        /* 波形直接采样 Motor 实时字段，不再经 App 镜像。 */
        {"Iu", 1000.0f, (void *)&ptThis->tMotor.qIuLatest},
        {"Iv", 1000.0f, (void *)&ptThis->tMotor.qIvLatest},
        {"Iw", 1000.0f, (void *)&ptThis->tMotor.qIwLatest},
        {"Id", 1000.0f, (void *)&ptThis->tMotor.tCore.tCurrent.qD},
        {"Iq", 1000.0f, (void *)&ptThis->tMotor.tCore.tCurrent.qQ},
        {"Angle", 1000.0f,
         (void *)&ptThis->tDiagnostics.fElectricalAngleTurns},
        /* Speed scale 100：电速度可到 ±100 eHz，×100=10000 不超 int16；
           scale 1000 时 >32.8 eHz 即饱和卷绕（100→±32.7 假象）。 */
        {"Speed", 100.0f,
         (void *)&ptThis->tMotor.tPositionFeedback.qElectricalSpeed},
        {"Vd", 1000.0f, (void *)&ptThis->tMotor.tCore.tVoltage.qD},
        {"Vq", 1000.0f, (void *)&ptThis->tMotor.tCore.tVoltage.qQ},
    };
    uint8_t chIndex = 0U;
    bool bOk = true;
    int nResult = 0;
    uint32_t wActualRate = 0U;

    if (ptThis == NULL) {
        return;
    }
    nResult = mwaveform.Init(NULL);
    if (nResult != MODUS_SUCCESS) {
        MLOG(E, "[Waveform] init failed, disabled\r\n");
        return;
    }
    for (chIndex = 0U; chIndex < (uint8_t)(
             sizeof(atChannels) / sizeof(atChannels[0U]));
         chIndex++) {
        if (mwaveform.AddVariable(atChannels[chIndex].pchName,
                                  atChannels[chIndex].fScale,
                                  atChannels[chIndex].pvValue,
                                  MWAVEFORM_VAR_FLOAT) == (uint8_t)0xFFU) {
            bOk = false;
            break;
        }
    }
    ptThis->tDiagnostics.chEncoderWaveform = mwaveform.AddVariable(
        "EncMech", 1000.0f,
        (void *)&ptThis->tDiagnostics.fEncoderMechanicalTurns,
        MWAVEFORM_VAR_FLOAT);
    bOk = bOk &&
          (ptThis->tDiagnostics.chEncoderWaveform != (uint8_t)0xFFU);
    if (!bOk) {
        MLOG(E, "[Waveform] AddVariable failed, disabled\r\n");
        return;
    }
    wActualRate = mwaveform.SetStreamRate(50000U, 10000U);
    if (wActualRate == 0U) {
        MLOG(E, "[Waveform] stream rate failed, disabled\r\n");
        return;
    }
    mwaveform.SetRate(0U);
    wActualRate = mwaveform.SetChannelRate(
        ptThis->tDiagnostics.chEncoderWaveform, 1000U);
    if (wActualRate == 0U) {
        MLOG(E, "[Waveform] encoder rate failed, disabled\r\n");
        return;
    }
    mwaveform.Start();
    MLOG(I, "[Waveform] 10 channels registered, EncMech=1 kHz\r\n");
}
#endif

/**
 * @brief Initialize one complete FOC application object.
 * @param wObjectAddr Address of a foc_app_t object.
 * @param wObjectCfgAddr Address of a foc_app_cfg_t configuration.
 * @return FOC_RESULT_OK or an initialization error.
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    const foc_app_cfg_t *ptCfg = (const foc_app_cfg_t *)wObjectCfgAddr;
    motor_cfg_t tMotorCfg = {0};

    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL || ptCfg == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_app_ConfigValid(ptCfg)) {
        if (ptCfg->ptPwmOps != NULL) {
            ptCfg->ptPwmOps->fnEmergencyStop(NULL);
        }
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptThis, 0, sizeof(*ptThis));
    ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_OPEN_LOOP;
    ptThis->tPosition.tElectricalZero = ptCfg->tElectricalZero;
    ptThis->tPosition.chPolePairs = ptCfg->tEncoderParams.chPolePairs;
    ptThis->tPosition.bDirectionInverted = ptCfg->bDirectionInverted;
    ptThis->tSensor = ptCfg->tSensor;
    if (ptThis->tSensor.ptOps == NULL) {
        (void)foc_port_SensorInit(&ptCfg->tEncoderParams);
        ptThis->tSensor = g_tFocSensor;
    }
    ptThis->tPosition.bEncoderEnabled =
        (ptThis->tSensor.ptOps != NULL);
    tMotorCfg.tMotorParams.chPolePairs =
        ptCfg->tEncoderParams.chPolePairs;
    tMotorCfg.tControlCfg.tCurrentPiParams = ptCfg->tCurrentPiParams;
    tMotorCfg.tControlCfg.tSpeedPiParams = ptCfg->tSpeedPiParams;
    tMotorCfg.tControlCfg.qHighFrequencyPeriod =
        ptCfg->tEncoderParams.qHighFrequencyPeriod;
    tMotorCfg.tControlCfg.hwCalibrationTimeoutTicks =
        FOC_APP_CALIBRATION_MAX_TICKS;
    tMotorCfg.tControlCfg.wPositionCalibrationTicks =
        FOC_APP_ENCODER_CAL_TICKS;
    tMotorCfg.ptPwmOps = ptCfg->ptPwmOps;
    tMotorCfg.ptAdcOps = ptCfg->ptAdcOps;
    tMotorCfg.tPosition.ptOps = &s_tFocAppMotorPositionOps;
    tMotorCfg.tPosition.pContext = ptThis;
    eResult = motor_Init(&ptThis->tMotor, &tMotorCfg);
    if (eResult != FOC_RESULT_OK) {
        ptCfg->ptPwmOps->fnEmergencyStop(NULL);
        return eResult;
    }

#if defined(MODUS_ENABLE) && MODUS_ENABLE
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;
    s_tFocAppBaseCfg.pchRingBuffer = ptCfg->pchRingBuffer;
    s_tFocAppBaseCfg.hwRingSize = ptCfg->hwRingSize;
#endif

#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    ptThis->tDiagnostics.chEncoderWaveform = (uint8_t)0xFFU;
#endif
#if defined(MODUS_ENABLE) && MODUS_ENABLE
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    foc_app_WaveformInit(ptThis);
#endif
    eResult = (foc_result_t)mbase_Init(ptThis->ptBase,
                                       &s_tFocAppBaseCfg);
    if (eResult != FOC_RESULT_OK) {
        ptCfg->ptPwmOps->fnEmergencyStop(NULL);
        return eResult;
    }
#endif
    return FOC_RESULT_OK;
}

/**
 * @brief Start an App object in the requested control mode.
 * @param ptThis FOC application object.
 * @param ptCommand Control mode and reference values.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_Start(foc_app_t *ptThis,
                           const foc_core_command_t *ptCommand)
{
    if (ptThis == NULL || ptCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_app_ModeValid(ptCommand->eMode)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (ptCommand->eMode == FOC_MODE_SPEED &&
        !foc_app_EncoderReady(ptThis)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    /* 位置源选择只作用于 App 的 provider 上下文（开环/编码器）。 */
    ptThis->tPosition.bCurrentStartupAlign = false;
    ptThis->tPosition.wCurrentAlignTicks = 0U;
    if (ptCommand->eMode == FOC_MODE_CURRENT) {
        if (foc_app_EncoderReady(ptThis)) {
            ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_ENCODER;
        } else {
            ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_OPEN_LOOP;
            ptThis->tPosition.bCurrentStartupAlign = true;
        }
        ptThis->tPosition.qOpenLoopSpeed = FOC_ZERO;
    } else if (ptCommand->eMode == FOC_MODE_SPEED) {
        ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_ENCODER;
    } else {
        /* Voltage mode keeps the configured open-loop position source. */
    }
    if (motor_Start(&ptThis->tMotor, ptCommand) != FOC_RESULT_OK) {
        return FOC_RESULT_BUSY;
    }
    foc_app_SyncMotorView(ptThis);
    return FOC_RESULT_OK;
}

/**
 * @brief Stop an App object immediately and publish the lifecycle change.
 * @param ptThis FOC application object.
 * @return None.
 */
void foc_app_Stop(foc_app_t *ptThis)
{
    if (ptThis != NULL) {
        motor_Stop(&ptThis->tMotor);
        foc_app_SyncMotorView(ptThis);
    }
}

/**
 * @brief Request fault clearing after the power stage is disabled.
 * @param ptThis FOC application object.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_ClearFault(foc_app_t *ptThis)
{
    foc_result_t eResult = FOC_RESULT_BUSY;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = motor_ClearFault(&ptThis->tMotor);
    if (eResult == FOC_RESULT_OK) {
        foc_app_SyncMotorView(ptThis);
    }
    return eResult;
}

/**
 * @brief Set the voltage reference of a voltage-mode App object.
 * @param ptThis FOC application object.
 * @param qD D-axis voltage reference.
 * @param qQ Q-axis voltage reference.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetVoltageReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = motor_SetVoltageReference(&ptThis->tMotor, qD, qQ);
    foc_app_SyncMotorView(ptThis);
    return eResult;
}

/**
 * @brief Set the current reference of a current-mode App object.
 * @param ptThis FOC application object.
 * @param qD D-axis current reference.
 * @param qQ Q-axis current reference.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetCurrentReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = motor_SetCurrentReference(&ptThis->tMotor, qD, qQ);
    foc_app_SyncMotorView(ptThis);
    return eResult;
}

/**
 * @brief Set the electrical speed reference of speed mode.
 * @param ptThis FOC application object.
 * @param qElectricalTurnPerSecond Electrical speed in e-turn/s.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetSpeedReference(
    foc_app_t *ptThis, foc_scalar_t qElectricalTurnPerSecond)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    eResult = motor_SetSpeedReference(
        &ptThis->tMotor, qElectricalTurnPerSecond);
    foc_app_SyncMotorView(ptThis);
    return eResult;
}

/**
 * @brief Configure an open-loop position source before starting.
 * @param ptThis FOC application object.
 * @param tInitialAngle Initial electrical angle.
 * @param qElectricalSpeed Open-loop electrical speed in e-turn/s.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_ConfigureOpenLoop(
    foc_app_t *ptThis,
    foc_angle_t tInitialAngle,
    foc_scalar_t qElectricalSpeed)
{
    motor_status_t tStatus = {0};
    uintptr_t wState = 0U;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (motor_GetStatus(&ptThis->tMotor, &tStatus) != FOC_RESULT_OK) {
        return FOC_RESULT_BUSY;
    }
    if (tStatus.eLifecycle == MOTOR_STATE_RUNNING ||
        tStatus.eLifecycle == MOTOR_STATE_POSITION_CAL ||
        tStatus.eLifecycle == MOTOR_STATE_FAULT ||
        tStatus.wFaults != 0U) {
        return FOC_RESULT_BUSY;
    }
    wState = perfc_port_disable_global_interrupt();
    ptThis->tPosition.eAngleSource = FOC_APP_ANGLE_OPEN_LOOP;
    ptThis->tPosition.tOpenLoopAngle = tInitialAngle;
    ptThis->tPosition.qOpenLoopSpeed = qElectricalSpeed;
    ptThis->tPosition.qOpenLoopTargetSpeed = qElectricalSpeed;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

/**
 * @brief Submit an encoder zero calibration request through Motor.
 * @param ptThis FOC application object.
 * @return FOC_RESULT_OK or an error code.
 * @note Motor owns the align/capture state machine and drives the safe
 *       align current; this wrapper only keeps the encoder availability
 *       gate and mirrors the Motor lifecycle back into the App view.
 */
foc_result_t foc_app_RequestEncoderCalibration(foc_app_t *ptThis)
{
    foc_result_t eResult = FOC_RESULT_BUSY;

    if (ptThis == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->tSensor.ptOps == NULL ||
        !ptThis->tPosition.bEncoderEnabled) {
        return FOC_RESULT_BUSY;
    }
    eResult = motor_RequestPositionCalibration(
        &ptThis->tMotor, FOC_SCALAR(FOC_APP_ENC_CAL_ID_ALIGN));
    if (eResult == FOC_RESULT_OK) {
        foc_app_SyncMotorView(ptThis);
    }
    return eResult;
}

/**
 * @brief Read the primary position feedback (mechanical angle/speed).
 * @param ptThis           FOC application object.
 * @param ptMechanicalAngle Output mechanical angle in turns.
 * @param pqMechanicalSpeed Output mechanical speed in turn/s.
 * @param pbValid           Output feedback validity.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_GetFeedback(const foc_app_t *ptThis,
                                 foc_angle_t *ptMechanicalAngle,
                                 foc_scalar_t *pqMechanicalSpeed,
                                 bool *pbValid)
{
    if (ptThis == NULL || ptMechanicalAngle == NULL ||
        pqMechanicalSpeed == NULL || pbValid == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptThis->tSensor.ptOps == NULL ||
        ptThis->tSensor.ptOps->fnRead == NULL) {
        return FOC_RESULT_DISABLED;
    }
    return ptThis->tSensor.ptOps->fnRead(
        ptThis->tSensor.pPriv,
        ptMechanicalAngle,
        pqMechanicalSpeed,
        pbValid);
}

/**
 * @brief Map a Motor lifecycle onto the App query-state enum.
 * @param eLifecycle Motor lifecycle.
 * @return Matching foc_run_state_e value.
 */
static foc_run_state_e foc_app_MapState(motor_lifecycle_e eLifecycle)
{
    switch (eLifecycle) {
    case MOTOR_STATE_CALIBRATING:
        return FOC_STATE_CALIBRATING;
    case MOTOR_STATE_POSITION_CAL:
        return FOC_STATE_CALIBRATING;
    case MOTOR_STATE_RUNNING:
        return FOC_STATE_RUNNING;
    case MOTOR_STATE_FAULT:
        return FOC_STATE_FAULT;
    case MOTOR_STATE_IDLE:
    case MOTOR_STATE_INITIALIZING:
    default:
        return FOC_STATE_IDLE;
    }
}

/**
 * @brief Assemble a status snapshot directly from the Motor object.
 * @param ptThis FOC application object.
 * @param ptStatus Output status snapshot.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_GetStatus(const foc_app_t *ptThis,
                               foc_status_t *ptStatus)
{
    motor_feedback_t tFeedback = {0};
    motor_status_t tStatus = {0};
    uintptr_t wState = 0U;

    if (ptThis == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    wState = perfc_port_disable_global_interrupt();
    (void)motor_GetFeedback(&ptThis->tMotor, &tFeedback);
    (void)motor_GetStatus(&ptThis->tMotor, &tStatus);
    ptStatus->eState = foc_app_MapState(tStatus.eLifecycle);
    ptStatus->wFaults = tStatus.wFaults;
    ptStatus->tElectricalAngle = tFeedback.tPosition.tElectricalAngle;
    ptStatus->tElectricalZero = ptThis->tPosition.tElectricalZero;
    ptStatus->qElectricalSpeed = tFeedback.tPosition.qElectricalSpeed;
    ptStatus->tCurrent = tFeedback.tCurrent;
    ptStatus->tVoltage = tFeedback.tVoltage;
    ptStatus->tDuty = tFeedback.tDuty;
    ptStatus->tCalibration = ptThis->tMotor.tAdcCalibration;
    ptStatus->eMode = tStatus.tCommand.eMode;
    ptStatus->tVoltageReference = tStatus.tCommand.tVoltageReference;
    ptStatus->tCurrentReference = tStatus.tCommand.tCurrentReference;
    ptStatus->qSpeedReference = tStatus.tCommand.qSpeedReference;
    ptStatus->bPwmEnabled = tStatus.bPwmEnabled;
    ptStatus->bEncoderCalibrated = ptThis->tPosition.bEncoderCalibrated;
    perfc_port_resume_global_interrupt(wState);
    return FOC_RESULT_OK;
}

#if defined(FOC_APP_TEST)
/**
 * @brief Mark an App test object as encoder-calibrated.
 * @param ptThis FOC application object.
 * @return None.
 */
void foc_app_TestMarkEncoderCalibrated(foc_app_t *ptThis)
{
    if (ptThis != NULL) {
        ptThis->tPosition.bEncoderCalibrated = true;
    }
}

/**
 * @brief Run the object speed-loop test hook.
 * @param ptThis FOC application object.
 * @return None.
 */
void foc_app_TestRun1kHz(foc_app_t *ptThis)
{
    motor_status_t tStatus = {0};

    if (ptThis == NULL) {
        return;
    }
    if (motor_GetStatus(&ptThis->tMotor, &tStatus) != FOC_RESULT_OK) {
        return;
    }
    if (tStatus.eLifecycle == MOTOR_STATE_RUNNING &&
        tStatus.tCommand.eMode == FOC_MODE_SPEED) {
        motor_ClockStep(&ptThis->tMotor);
    }
}

/**
 * @brief Read a test object's commanded Q-axis current.
 * @param ptThis FOC application object.
 * @return Current Q-axis reference in per-unit.
 */
foc_scalar_t foc_app_TestGetCurrentIqReference(
    const foc_app_t *ptThis)
{
    if (ptThis == NULL) {
        return FOC_ZERO;
    }
    return ptThis->tMotor.tCommand.tCurrentReference.qQ;
}
#endif

void foc_app_HighFrequencyISR(void)
{
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    foc_app_HighFrequencyStep(&tFocApp);
#endif
}

#if defined(MODUS_ENABLE) && MODUS_ENABLE
/**
 * @brief Convert a shell float into a bounded test reference.
 * @param fValue Input value.
 * @param fMinimum Lower bound.
 * @param fMaximum Upper bound.
 * @return Bounded value.
 */
static float foc_app_Clamp(float fValue,
                           float fMinimum,
                           float fMaximum)
{
    float fClamped = fValue;

    if (fClamped < fMinimum) {
        fClamped = fMinimum;
    } else if (fClamped > fMaximum) {
        fClamped = fMaximum;
    } else {
        /* The requested value is already inside the safe range. */
    }
    return fClamped;
}

/**
 * @brief Return a printable lifecycle name.
 * @param eState Lifecycle state.
 * @return Constant state name.
 */
static const char *foc_app_StateName(foc_run_state_e eState)
{
    switch (eState) {
    case FOC_STATE_IDLE:
        return "IDLE";
    case FOC_STATE_CALIBRATING:
        return "CALIBRATING";
    case FOC_STATE_RUNNING:
        return "RUNNING";
    case FOC_STATE_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

/**
 * @brief Check that a shell command may start the App.
 * @param ptThis FOC application object.
 * @return true when the object is idle without faults.
 */
static bool foc_app_ShellIsIdle(foc_app_t *ptThis)
{
    foc_status_t tStatus = {0};

    if (foc_app_GetStatus(ptThis, &tStatus) != FOC_RESULT_OK) {
        return false;
    }
    return tStatus.eState == FOC_STATE_IDLE && tStatus.wFaults == 0U;
}

/**
 * @brief Parse two bounded floats with defaults from a shell argument tail.
 * @param pchArgs Argument tail after the command keyword.
 * @param atSpec  Two-element parameter spec (default/min/max per value).
 * @param pfFirst Output first value.
 * @param pfSecond Output second value.
 * @return None.
 */
typedef struct {
    float fDefault;
    float fMin;
    float fMax;
} foc_app_shell_spec_t;

static void foc_app_ShellParseTwo(const char *pchArgs,
                                  const foc_app_shell_spec_t atSpec[2],
                                  float *pfFirst,
                                  float *pfSecond)
{
    int nScanned = 0;

    nScanned = sscanf(pchArgs, "%f %f", pfFirst, pfSecond);
    if (nScanned < 1) {
        *pfFirst = atSpec[0].fDefault;
    }
    if (nScanned < 2) {
        *pfSecond = atSpec[1].fDefault;
    }
    *pfFirst = foc_app_Clamp(*pfFirst, atSpec[0].fMin, atSpec[0].fMax);
    *pfSecond = foc_app_Clamp(*pfSecond, atSpec[1].fMin, atSpec[1].fMax);
}

/**
 * @brief Start a voltage-mode open-loop command from the shell.
 * @param ptThis FOC application object.
 * @param tAngle Initial electrical angle.
 * @param qSpeed Electrical open-loop speed.
 * @param tVoltage Voltage reference.
 * @return FOC_RESULT_OK or an error code.
 */
static foc_result_t foc_app_StartVoltage(
    foc_app_t *ptThis,
    foc_angle_t tAngle,
    foc_scalar_t qSpeed,
    foc_dq_t tVoltage)
{
    foc_core_command_t tCommand = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_app_ConfigureOpenLoop(ptThis, tAngle, qSpeed);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    tCommand.eMode = FOC_MODE_VOLTAGE;
    tCommand.tVoltageReference = tVoltage;
    return foc_app_Start(ptThis, &tCommand);
}

/**
 * @brief Start a current-mode command from the shell.
 * @param ptThis FOC application object.
 * @param qIq Q-axis current reference.
 * @param qSpeed Open-loop fallback speed.
 * @return FOC_RESULT_OK or an error code.
 */
static foc_result_t foc_app_StartCurrent(foc_app_t *ptThis,
                                         foc_scalar_t qIq,
                                         foc_scalar_t qSpeed)
{
    foc_core_command_t tCommand = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = foc_app_ConfigureOpenLoop(
        ptThis, (foc_angle_t){0U}, qSpeed);
    if (eResult != FOC_RESULT_OK) {
        return eResult;
    }
    tCommand.eMode = FOC_MODE_CURRENT;
    tCommand.tCurrentReference.qD = FOC_ZERO;
    tCommand.tCurrentReference.qQ = qIq;
    return foc_app_Start(ptThis, &tCommand);
}

/**
 * @brief Start the encoder speed-mode command from the shell.
 * @param ptThis FOC application object.
 * @param qIq Maximum Q-axis current reference.
 * @param qSpeed Electrical speed reference.
 * @return FOC_RESULT_OK or an error code.
 */
static foc_result_t foc_app_StartSpeed(foc_app_t *ptThis,
                                       foc_scalar_t qIq,
                                       foc_scalar_t qSpeed)
{
    foc_core_command_t tCommand = {0};

    tCommand.eMode = FOC_MODE_SPEED;
    tCommand.tCurrentReference.qD = FOC_ZERO;
    tCommand.tCurrentReference.qQ = qIq;
    tCommand.qSpeedReference = qSpeed;
    return foc_app_Start(ptThis, &tCommand);
}

/**
 * @brief Print the status snapshot requested by the shell.
 * @param ptThis FOC application object.
 * @return None.
 */
static void foc_app_CmdStatus(foc_app_t *ptThis)
{
    foc_status_t tStatus = {0};

    if (foc_app_GetStatus(ptThis, &tStatus) != FOC_RESULT_OK) {
        MLOG(E, "motor status failed\r\n");
        return;
    }
    MLOGF(I, "state=%s faults=0x%lX pwm=%d enc_cal=%d\r\n",
          foc_app_StateName(tStatus.eState),
          (unsigned long)tStatus.wFaults,
          (int)tStatus.bPwmEnabled,
          (int)tStatus.bEncoderCalibrated);
    MLOGF(I, "angle=%.1f deg speed=%.4f e-turn/s\r\n",
          (double)foc_angle_to_turns(
              tStatus.tElectricalAngle) * 360.0,
          (double)foc_to_float(tStatus.qElectricalSpeed));
    MLOGF(I, "Id=%.4f Iq=%.4f | Vd=%.4f Vq=%.4f\r\n",
          (double)foc_to_float(tStatus.tCurrent.qD),
          (double)foc_to_float(tStatus.tCurrent.qQ),
          (double)foc_to_float(tStatus.tVoltage.qD),
          (double)foc_to_float(tStatus.tVoltage.qQ));
    MLOGF(I, "duty U=%.3f V=%.3f W=%.3f\r\n",
          (double)foc_to_float(tStatus.tDuty.qU),
          (double)foc_to_float(tStatus.tDuty.qV),
          (double)foc_to_float(tStatus.tDuty.qW));
}

/**
 * @brief Adjust one voltage reference without accessing object internals.
 * @param ptThis FOC application object.
 * @param args Shell argument after vd/vq.
 * @param bD true for Vd, false for Vq.
 * @return None.
 */
static void foc_app_CmdAdjustVoltage(foc_app_t *ptThis,
                                     const char *args,
                                     bool bD)
{
    foc_status_t tStatus = {0};
    foc_scalar_t qD = FOC_ZERO;
    foc_scalar_t qQ = FOC_SCALAR(0.03f);
    float fValue = 0.0f;
    int nScanned = 0;
    foc_result_t eResult = FOC_RESULT_OK;

    if (foc_app_GetStatus(ptThis, &tStatus) != FOC_RESULT_OK) {
        return;
    }
    qD = tStatus.tVoltageReference.qD;
    qQ = tStatus.tVoltageReference.qQ;
    nScanned = sscanf(args, "%f", &fValue);
    if (nScanned == 1) {
        fValue = foc_app_Clamp(fValue, -0.30f, 0.30f);
        if (bD) {
            qD = FOC_SCALAR(fValue);
        } else {
            qQ = FOC_SCALAR(fValue);
        }
    }
    eResult = foc_app_SetVoltageReference(ptThis, qD, qQ);
    if (eResult != FOC_RESULT_OK) {
        MLOG(W, "voltage reference rejected\r\n");
    }
}

/* 各 motor 子命令的两个浮点参数规格：默认值 + 安全钳位边界。 */
static const foc_app_shell_spec_t s_atLockSpec[2] = {
    {0.0f, -1000.0f, 1000.0f},   /* 锁角 (turns) */
    {0.0f, -1000.0f, 1000.0f},   /* 未使用 */
};
static const foc_app_shell_spec_t s_atSpinSpec[2] = {
    {2.0f, -100.0f, 100.0f},     /* 开环速度 (e-turn/s) */
    {0.03f, -0.30f, 0.30f},      /* Vq (pu) */
};
static const foc_app_shell_spec_t s_atCurrentSpec[2] = {
    {0.05f, 0.02f, 0.15f},       /* Iq (pu) */
    {0.0f, -100.0f, 100.0f},     /* 开环回退速度 */
};
static const foc_app_shell_spec_t s_atEncSpec[2] = {
    {0.10f, 0.02f, 0.15f},       /* Iq (pu) */
    {50.0f, -100.0f, 100.0f},    /* 速度参考 */
};

/**
 * @brief Parse and start the motor shell command family.
 * @param args Full shell argument string.
 * @return None.
 */
static void foc_app_CmdMotor(const char *args)
{
    foc_status_t tStatus = {0};
    float fFirst = 0.0f;
    float fSecond = 0.0f;
    foc_result_t eResult = FOC_RESULT_OK;

    if (args == NULL) {
        return;
    }
    if (strncmp(args, "start", 5) == 0) {
        if (!foc_app_ShellIsIdle(&tFocApp)) {
            MLOG(W, "motor start rejected (not IDLE)\r\n");
            return;
        }
        eResult = foc_app_StartVoltage(
            &tFocApp, (foc_angle_t){0U}, FOC_SCALAR(2.0f),
            (foc_dq_t){FOC_ZERO, FOC_SCALAR(0.03f)});
    } else if (strncmp(args, "lock", 4) == 0) {
        foc_app_ShellParseTwo(args + 4, s_atLockSpec,
                              &fFirst, &fSecond);
        if (!foc_app_ShellIsIdle(&tFocApp)) {
            MLOG(W, "motor lock rejected (not IDLE)\r\n");
            return;
        }
        eResult = foc_app_StartVoltage(
            &tFocApp, foc_angle_from_turns(fFirst), FOC_ZERO,
            (foc_dq_t){FOC_SCALAR(0.04f), FOC_ZERO});
    } else if (strncmp(args, "spin", 4) == 0) {
        foc_app_ShellParseTwo(args + 4, s_atSpinSpec,
                              &fFirst, &fSecond);
        if (!foc_app_ShellIsIdle(&tFocApp)) {
            MLOG(W, "motor spin rejected (not IDLE)\r\n");
            return;
        }
        eResult = foc_app_StartVoltage(
            &tFocApp, (foc_angle_t){0U}, FOC_SCALAR(fFirst),
            (foc_dq_t){FOC_ZERO, FOC_SCALAR(fSecond)});
    } else if (strncmp(args, "current", 7) == 0) {
        foc_app_ShellParseTwo(args + 7, s_atCurrentSpec,
                              &fFirst, &fSecond);
        if (foc_app_GetStatus(&tFocApp, &tStatus) != FOC_RESULT_OK ||
            !tStatus.bEncoderCalibrated) {
            MLOG(W, "motor current requires: encoder cal\r\n");
            return;
        }
        if (!foc_app_ShellIsIdle(&tFocApp)) {
            MLOG(W, "motor current rejected (not IDLE)\r\n");
            return;
        }
        eResult = foc_app_StartCurrent(
            &tFocApp, FOC_SCALAR(fFirst), FOC_SCALAR(fSecond));
    } else if (strncmp(args, "enc", 3) == 0) {
        foc_app_ShellParseTwo(args + 3, s_atEncSpec,
                              &fFirst, &fSecond);
        if (!foc_app_ShellIsIdle(&tFocApp)) {
            MLOG(W, "motor enc rejected (not IDLE)\r\n");
            return;
        }
        eResult = foc_app_StartSpeed(
            &tFocApp, FOC_SCALAR(fFirst), FOC_SCALAR(fSecond));
    } else if (strncmp(args, "stop", 4) == 0) {
        foc_app_Stop(&tFocApp);
        return;
    } else if (strncmp(args, "clear", 5) == 0) {
        eResult = foc_app_ClearFault(&tFocApp);
    } else if (strncmp(args, "status", 6) == 0) {
        foc_app_CmdStatus(&tFocApp);
        return;
    } else if (strncmp(args, "timing", 6) == 0) {
        foc_app_IsrTimingPrint(&tFocApp, false);
        return;
    } else if (strncmp(args, "vd", 2) == 0) {
        foc_app_CmdAdjustVoltage(&tFocApp, args + 2, true);
        return;
    } else if (strncmp(args, "vq", 2) == 0) {
        foc_app_CmdAdjustVoltage(&tFocApp, args + 2, false);
        return;
    } else {
        MLOG(I, "usage: motor start|lock [turns]|spin <hz> <vq>|"
                "current <iq> <hz>|enc <iq> <hz>\r\n");
        MLOG(I, "       motor vd <x>|vq <x>|stop|clear|status|timing\r\n");
        return;
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "motor command rejected (%d)\r\n", (int)eResult);
    }
}
MODUS_SHELL_CMD(motor, foc_app_CmdMotor,
                "FOC test: start/lock/spin/current/enc/vd/vq/"
                "stop/status/timing");

/**
 * @brief Print or request the position feedback operation.
 * @param args Encoder shell arguments.
 * @return None.
 */
static void cmd_encoder(const char *args)
{
    foc_angle_t tMechanicalAngle = {0U};
    foc_scalar_t qMechanicalSpeed = FOC_ZERO;
    bool bValid = false;
    foc_status_t tStatus = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (args != NULL && strncmp(args, "cal", 3) == 0) {
        eResult = foc_app_RequestEncoderCalibration(&tFocApp);
        if (eResult == FOC_RESULT_OK) {
            MLOG(I, "Encoder cal requested; keep shaft untouched\r\n");
        } else {
            MLOGF(W, "encoder cal request rejected (%d)\r\n",
                  (int)eResult);
        }
        return;
    }
    eResult = foc_app_GetFeedback(&tFocApp, &tMechanicalAngle,
                                  &qMechanicalSpeed, &bValid);
    if (eResult != FOC_RESULT_OK) {
        MLOG(E, "encoder sample unavailable\r\n");
        return;
    }
    eResult = foc_app_GetStatus(&tFocApp, &tStatus);
    if (eResult != FOC_RESULT_OK) {
        MLOG(E, "encoder status unavailable\r\n");
        return;
    }
    MLOGF(I, "feedback: mech=%.4f turn (%.2f deg) speed=%.4f "
             "valid=%d\r\n",
          (double)foc_angle_to_turns(tMechanicalAngle),
          (double)foc_angle_to_turns(tMechanicalAngle) * 360.0,
          (double)foc_to_float(qMechanicalSpeed),
          (int)bValid);
    MLOGF(I, "  elec-offset=%.4f turn calibrated=%d\r\n",
          (double)foc_angle_to_turns(tStatus.tElectricalZero),
          (int)tStatus.bEncoderCalibrated);
}
MODUS_SHELL_CMD(encoder, cmd_encoder,
                "Show position feedback; cal = zero offset");
#endif
