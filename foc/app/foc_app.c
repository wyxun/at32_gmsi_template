/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS and product glue for the single Motor object.
 * @author  Codex
 * @date    2026-09-09
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_app.h"
#include "foc_config.h"
#include "foc_port.h"
#include "mdebug/util_debug.h"

#if defined(MODUS_ENABLE) && MODUS_ENABLE
#include "mdebug/mshell.h"
#include "perfc_task_pt.h"
#endif

#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
#include "mdebug/mwaveform.h"
#endif

#define FOC_APP_HF_PERIOD_S 0.00005f
#define FOC_APP_ADC_CAL_TICKS 2000U
#define FOC_APP_ALIGN_TICKS 30000U
#define FOC_APP_ALIGN_CURRENT 0.10f

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

MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tMotorConfig = {
        .tMotorParams = {
            .chPolePairs = 7U,
        },
        .tControlCfg = {
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
            .qHighFrequencyPeriod = FOC_SCALAR(FOC_APP_HF_PERIOD_S),
            .qPositionCalibrationCurrent =
                FOC_SCALAR(FOC_APP_ALIGN_CURRENT),
            .qSpeedIqLimit = FOC_SCALAR(0.10f),
            .hwCalibrationTimeoutTicks = FOC_APP_ADC_CAL_TICKS,
            .wPositionCalibrationTicks = FOC_APP_ALIGN_TICKS,
        },
        .tEncoderParams = {
            .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
            .hwInvalidTimeout = 100U,
        },
        .ptAdcOps = &g_tFocAdcOps,
        .ptPwmOps = &g_tFocPwmOps,
        .tPosition = FOC_PORT_DEFAULT_POSITION,
    },
)
#endif

static bool foc_app_ConfigValid(const foc_app_cfg_t *ptConfig)
{
    if (ptConfig == NULL || ptConfig->tMotorConfig.ptAdcOps == NULL ||
        ptConfig->tMotorConfig.ptPwmOps == NULL ||
        ptConfig->tMotorConfig.tMotorParams.chPolePairs == 0U ||
        ptConfig->tMotorConfig.tControlCfg.qHighFrequencyPeriod <=
            FOC_ZERO) {
        return false;
    }
    return true;
}

#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
static void foc_app_WaveformInit(foc_app_t *ptThis)
{
    static const char *const asNames[] = {
        "Iu", "Iv", "Iw", "Id", "Iq", "Speed", "Vd", "Vq"
    };
    void *apValues[8] = {0};
    float afScales[8] = {1000.0f, 1000.0f, 1000.0f, 1000.0f,
                         1000.0f, 100.0f, 1000.0f, 1000.0f};
    uint8_t chIndex = 0U;
    bool bOk = true;
    int nResult = 0;

    if (ptThis == NULL) {
        return;
    }
    apValues[0] = (void *)&ptThis->tMotor.tControl.tCycleInput.qIu;
    apValues[1] = (void *)&ptThis->tMotor.tControl.tCycleInput.qIv;
    apValues[2] = (void *)&ptThis->tMotor.tControl.tCycleInput.qIw;
    apValues[3] = (void *)&ptThis->tMotor.tControl.tCore.tCurrent.qD;
    apValues[4] = (void *)&ptThis->tMotor.tControl.tCore.tCurrent.qQ;
    apValues[5] = (void *)&ptThis->tMotor.tControl.tPositionFeedback.
                  qElectricalSpeed;
    apValues[6] = (void *)&ptThis->tMotor.tControl.tCore.tVoltage.qD;
    apValues[7] = (void *)&ptThis->tMotor.tControl.tCore.tVoltage.qQ;
    nResult = mwaveform.Init(NULL);
    if (nResult != MODUS_SUCCESS) {
        return;
    }
    for (chIndex = 0U; chIndex < 8U; chIndex++) {
        if (mwaveform.AddVariable(asNames[chIndex], afScales[chIndex],
                                  apValues[chIndex],
                                  MWAVEFORM_VAR_FLOAT) == 0xFFU) {
            bOk = false;
            break;
        }
    }
    if (!bOk) {
        return;
    }
    (void)mwaveform.SetStreamRate(50000U, 10000U);
    (void)mwaveform.SetRate(0U);
    mwaveform.Start();
}
#endif

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    const foc_app_cfg_t *ptConfig = (const foc_app_cfg_t *)wObjectCfgAddr;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!foc_app_ConfigValid(ptConfig)) {
        if (ptConfig->tMotorConfig.ptPwmOps != NULL &&
            ptConfig->tMotorConfig.ptPwmOps->fnEmergencyStop != NULL) {
            ptConfig->tMotorConfig.ptPwmOps->fnEmergencyStop(
                ptConfig->tMotorConfig.pPwmContext);
        }
        return FOC_RESULT_INVALID_ARGUMENT;
    }
#if !FOC_PORT_HAS_POSITION
    ptConfig->tMotorConfig.ptPwmOps->fnEmergencyStop(
        ptConfig->tMotorConfig.pPwmContext);
    return FOC_RESULT_DISABLED;
#endif
    memset(ptThis, 0, sizeof(*ptThis));
    eResult = motor_Init(&ptThis->tMotor, &ptConfig->tMotorConfig);
    if (eResult != FOC_RESULT_OK) {
        ptConfig->tMotorConfig.ptPwmOps->fnEmergencyStop(
            ptConfig->tMotorConfig.pPwmContext);
        return eResult;
    }
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;
#endif
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    foc_app_WaveformInit(ptThis);
#endif
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    eResult = (foc_result_t)mbase_Init(ptThis->ptBase,
                                       &s_tFocAppBaseCfg);
    if (eResult != FOC_RESULT_OK) {
        ptConfig->tMotorConfig.ptPwmOps->fnEmergencyStop(
            ptConfig->tMotorConfig.pPwmContext);
        return eResult;
    }
#endif
    return FOC_RESULT_OK;
}

void foc_app_HighFrequencyStep(foc_app_t *ptThis)
{
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    int64_t lCycles = 0;
    uint32_t wCycles = 0U;

    start_cycle_counter();
#endif
    if (ptThis == NULL) {
        return;
    }
    motor_HighFrequencyStep(&ptThis->tMotor);
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    mwaveform.Step();
#endif
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    lCycles = stop_cycle_counter();
    if (lCycles > 0) {
        wCycles = lCycles > (int64_t)UINT32_MAX
                      ? UINT32_MAX : (uint32_t)lCycles;
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
static int foc_app_Clock(uintptr_t wObjectAddr)
{
    return wObjectAddr == 0U ? MODUS_EFAIL : MODUS_SUCCESS;
}

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->tDiagnostics.chRunPt)
    while (1) {
        motor_BackgroundStep(&ptThis->tMotor);
        PERFC_PT_YIELD(MODUS_SUCCESS);
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}

#if defined(MSHELL_ENABLE) && MSHELL_ENABLE
static const char *foc_app_StateName(motor_lifecycle_e eState)
{
    switch (eState) {
    case MOTOR_STATE_INITIALIZING:
        return "INITIALIZING";
    case MOTOR_STATE_ADC_CAL:
        return "ADC_CAL";
    case MOTOR_STATE_IDLE:
        return "IDLE";
    case MOTOR_STATE_ALIGN:
        return "ALIGN";
    case MOTOR_STATE_RUNNING:
        return "RUNNING";
    case MOTOR_STATE_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

static float foc_app_Clamp(float fValue, float fMin, float fMax)
{
    if (fValue < fMin) {
        return fMin;
    }
    if (fValue > fMax) {
        return fMax;
    }
    return fValue;
}

static void foc_app_CmdStatus(void)
{
    motor_status_t tStatus = {0};
    motor_feedback_t tFeedback = {0};

    if (motor_GetStatus(&tFocApp.tMotor, &tStatus) != FOC_RESULT_OK) {
        MLOG(E, "motor status failed\r\n");
        return;
    }
    if (motor_GetFeedback(&tFocApp.tMotor, &tFeedback) != FOC_RESULT_OK) {
        MLOG(E, "motor feedback failed\r\n");
        return;
    }
    MLOGF(I, "state=%s faults=0x%lX pwm=%d pos_cal=%d\r\n",
          foc_app_StateName(tStatus.eLifecycle),
          (unsigned long)tStatus.wFaults, (int)tStatus.bPwmEnabled,
          (int)tStatus.bPositionCalibrated);
    MLOGF(I, "angle=%.1f deg speed=%.4f e-turn/s\r\n",
          (double)foc_angle_to_turns(
              tFeedback.tPosition.tElectricalAngle) * 360.0,
          (double)foc_to_float(tFeedback.tPosition.qElectricalSpeed));
    MLOGF(I, "Id=%.4f Iq=%.4f | Vd=%.4f Vq=%.4f\r\n",
          (double)foc_to_float(tFeedback.tCurrent.qD),
          (double)foc_to_float(tFeedback.tCurrent.qQ),
          (double)foc_to_float(tFeedback.tVoltage.qD),
          (double)foc_to_float(tFeedback.tVoltage.qQ));
}

static void foc_app_CmdMotor(const char *args)
{
    float fFirst = 0.0f;
    float fSecond = 0.0f;
    foc_result_t eResult = FOC_RESULT_OK;

    if (args == NULL) {
        return;
    }
    if (strncmp(args, "start", 5) == 0) {
        eResult = motor_SetVoltageReference(
            &tFocApp.tMotor, FOC_ZERO, FOC_SCALAR(0.03f));
        if (eResult == FOC_RESULT_OK) {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE);
        }
    } else if (strncmp(args, "spin", 4) == 0) {
        (void)sscanf(args + 4, "%f %f", &fFirst, &fSecond);
        fFirst = foc_app_Clamp(fFirst, -100.0f, 100.0f);
        fSecond = foc_app_Clamp(fSecond, -0.30f, 0.30f);
        (void)fFirst;
        eResult = motor_SetVoltageReference(
            &tFocApp.tMotor, FOC_ZERO, FOC_SCALAR(fSecond));
        if (eResult == FOC_RESULT_OK) {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE);
        }
    } else if (strncmp(args, "current", 7) == 0) {
        (void)sscanf(args + 7, "%f %f", &fFirst, &fSecond);
        fFirst = foc_app_Clamp(fFirst, -0.15f, 0.15f);
        fSecond = foc_app_Clamp(fSecond, -100.0f, 100.0f);
        (void)fSecond;
        eResult = motor_SetCurrentReference(
            &tFocApp.tMotor, FOC_ZERO, FOC_SCALAR(fFirst));
        if (eResult == FOC_RESULT_OK) {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_CURRENT);
        }
    } else if (strncmp(args, "enc", 3) == 0) {
        (void)sscanf(args + 3, "%f", &fFirst);
        fFirst = foc_app_Clamp(fFirst, -100.0f, 100.0f);
        eResult = motor_SetSpeedReference(
            &tFocApp.tMotor, FOC_SCALAR(fFirst));
        if (eResult == FOC_RESULT_OK) {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_SPEED);
        }
    } else if (strncmp(args, "stop", 4) == 0) {
        motor_Stop(&tFocApp.tMotor);
        return;
    } else if (strncmp(args, "clear", 5) == 0) {
        eResult = motor_ClearFault(&tFocApp.tMotor);
    } else if (strncmp(args, "status", 6) == 0) {
        foc_app_CmdStatus();
        return;
    } else if (strncmp(args, "cal", 3) == 0) {
        eResult = motor_RequestPositionCalibration(&tFocApp.tMotor);
    } else {
        MLOG(I, "usage: motor start|spin <hz> <vq>|current <iq> <hz>|");
        MLOG(I, "enc <speed>|cal|stop|clear|status\r\n");
        return;
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "motor command rejected (%d)\r\n", (int)eResult);
    }
}

MODUS_SHELL_CMD(motor, foc_app_CmdMotor, "Motor control and diagnostics");
#endif
#endif

void foc_app_HighFrequencyISR(void)
{
#if defined(MODUS_ENABLE) && MODUS_ENABLE
    foc_app_HighFrequencyStep(&tFocApp);
#endif
}
