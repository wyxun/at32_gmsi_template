/****************************************************************************
 * @file    foc_app.c
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_app.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_port_config.h"
#include "mdebug/util_debug.h"
#include "perf_counter.h"
#include "perfc_task_pt.h"

#if MSHELL_ENABLE
#include "mdebug/mshell.h"
#endif

static modus_base_t s_tFocAppBase;
extern foc_app_t tFocApp;
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

/**
 * @brief Read a mechanical position through the owned Encoder object.
 * @param pContext Encoder object context.
 * @param wNowTick Current low 32-bit system tick.
 * @param ptPosition Output mechanical position.
 * @return Encoder result.
 */
static foc_result_t foc_app_GetPosition(void *pContext,
                                        uint32_t wNowTick,
                                        foc_position_t *ptPosition)
{
    return foc_encoder_GetPosition((const foc_encoder_t *)pContext,
                                   wNowTick, ptPosition);
}

/**
 * @brief Build the Motor config bound to this App's Encoder.
 * @param ptApp App object.
 * @param ptConfig App configuration.
 * @param ptMotorConfig Output Motor configuration.
 * @param bEncoderReady Whether the Encoder was initialized.
 * @return None.
 */
static void foc_app_BindMotorConfig(foc_app_t *ptApp,
                                    const foc_app_cfg_t *ptConfig,
                                    motor_cfg_t *ptMotorConfig,
                                    bool bEncoderReady)
{
    *ptMotorConfig = ptConfig->tMotorCfg;
    if (bEncoderReady) {
        ptMotorConfig->fnGetPosition = foc_app_GetPosition;
        ptMotorConfig->pPositionContext = &ptApp->tEncoder;
    } else {
        ptMotorConfig->fnGetPosition = NULL;
        ptMotorConfig->pPositionContext = NULL;
    }
}

int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_app_cfg_t *ptConfig = (foc_app_cfg_t *)wObjectCfgAddr;
    motor_cfg_t tMotorConfig = {0};
    foc_encoder_cfg_t tEncoderConfig = {0};
    foc_result_t eEncoder = FOC_RESULT_OK;
    foc_result_t eMotor = FOC_RESULT_OK;
    int nBaseResult = MODUS_SUCCESS;

    if (ptThis == NULL || ptConfig == NULL) {
        return MODUS_EFAIL;
    }
    *ptThis = (foc_app_t){0};
    ptThis->ptBase = &s_tFocAppBase;
    s_tFocAppBaseCfg.wParent = wObjectAddr;
    ptThis->chRunPt = 0U;
    ptThis->lForegroundTimestamp = 0;
    ptThis->lBackoffTimestamp = 0;
    ptThis->bReady = false;

    tEncoderConfig = ptConfig->tEncoderCfg;
#if FOC_PORT_HAS_POSITION
    if (tEncoderConfig.pSensorContext == NULL) {
        tEncoderConfig.pSensorContext = foc_port_PositionContext();
    }
#endif
    eEncoder = foc_encoder_Init(&ptThis->tEncoder,
                                &tEncoderConfig);
    if (eEncoder != FOC_RESULT_OK && eEncoder != FOC_RESULT_DISABLED) {
        foc_pwm_Stop();
        return (int)eEncoder;
    }
    foc_app_BindMotorConfig(ptThis, ptConfig, &tMotorConfig,
                            eEncoder == FOC_RESULT_OK);
    eMotor = motor_Init(&ptThis->tMotor, &tMotorConfig);
    if (eMotor != FOC_RESULT_OK) {
        foc_pwm_Stop();
        return (int)eMotor;
    }
    nBaseResult = mbase_Init(ptThis->ptBase, &s_tFocAppBaseCfg);
    if (nBaseResult != MODUS_SUCCESS) {
        foc_pwm_Stop();
        return nBaseResult;
    }
    ptThis->bReady = eEncoder == FOC_RESULT_OK;
    return MODUS_SUCCESS;
}

static int foc_app_Clock(uintptr_t wObjectAddr)
{
    if (wObjectAddr == (uintptr_t)0U) {
        return MODUS_EFAIL;
    }
    return MODUS_SUCCESS;
}

static int foc_app_Run(uintptr_t wObjectAddr)
{
    foc_app_t *ptThis = (foc_app_t *)wObjectAddr;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptThis == NULL) {
        return MODUS_EFAIL;
    }
    PERFC_PT_BEGIN(ptThis->chRunPt)
    while (true) {
        PERFC_PT_WAIT_UNTIL(perfc_is_time_out_us(
            1000U, &ptThis->lForegroundTimestamp, true))
        if (!ptThis->bReady) {
            continue;
        }
        if (ptThis->lBackoffTimestamp != 0 &&
            !perfc_is_time_out_ms(100U,
                                  &ptThis->lBackoffTimestamp, false)) {
            continue;
        }
        eResult = foc_encoder_Update(&ptThis->tEncoder);
        if (eResult != FOC_RESULT_OK) {
            ptThis->lBackoffTimestamp = get_system_ticks() +
                perfc_convert_ms_to_ticks(100U);
            continue;
        }
        ptThis->lBackoffTimestamp = 0;
    }
    PERFC_PT_END()
    return MODUS_SUCCESS;
}

void foc_app_HighFrequencyISR(void)
{
    uint32_t wNowTick = 0U;

    if (!tFocApp.bReady) {
        return;
    }
    wNowTick = (uint32_t)get_system_ticks();
    motor_HighFrequencyStep(&tFocApp.tMotor, wNowTick);
}

#if MSHELL_ENABLE
/**
 * @brief Print the Motor status without touching its internal members.
 * @param ptMotor Motor object.
 * @return None.
 */
static void foc_app_PrintStatus(const motor_t *ptMotor)
{
    motor_status_t tStatus = {0};
    foc_result_t eResult = motor_GetStatus(ptMotor, &tStatus);

    if (eResult != FOC_RESULT_OK) {
        MLOGF(E, "motor status unavailable (%d)\r\n", (int)eResult);
        return;
    }
    MLOGF(I, "motor state=%u fault=0x%08X mode=%u pwm=%u\r\n",
          (unsigned)tStatus.eState, (unsigned)tStatus.wFaults,
          (unsigned)tStatus.eMode, (unsigned)tStatus.bPwmEnabled);
}

/**
 * @brief Parse and submit one Motor command family command.
 * @param args Command arguments after the motor command name.
 * @return None.
 */
static void foc_app_CmdMotor(const char *args)
{
    float fD = 0.0f;
    float fQ = 0.0f;
    int nScanned = 0;
    bool bStarted = false;
    foc_result_t eResult = FOC_RESULT_OK;

    if (args == NULL) {
        return;
    }
    if (strncmp(args, "stop", 4U) == 0) {
        motor_Stop(&tFocApp.tMotor);
        return;
    } else if (strncmp(args, "clear", 5U) == 0) {
        eResult = motor_ClearFault(&tFocApp.tMotor);
    } else if (strncmp(args, "align", 5U) == 0) {
        eResult = motor_RequestPositionCalibration(&tFocApp.tMotor);
    } else if (strncmp(args, "speed", 5U) == 0) {
        nScanned = sscanf(args + 5, "%f", &fQ);
        if (nScanned != 1) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_SPEED);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetSpeedReference(&tFocApp.tMotor,
                                                  foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "current", 7U) == 0) {
        nScanned = sscanf(args + 7, "%f %f", &fD, &fQ);
        if (nScanned != 2) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_CURRENT);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetCurrentReference(&tFocApp.tMotor,
                                                    foc_from_float(fD),
                                                    foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "voltage", 7U) == 0) {
        nScanned = sscanf(args + 7, "%f %f", &fD, &fQ);
        if (nScanned != 2) {
            eResult = FOC_RESULT_INVALID_ARGUMENT;
        } else {
            eResult = motor_Start(&tFocApp.tMotor, FOC_MODE_VOLTAGE);
            bStarted = eResult == FOC_RESULT_OK;
            if (eResult == FOC_RESULT_OK) {
                eResult = motor_SetVoltageReference(&tFocApp.tMotor,
                                                    foc_from_float(fD),
                                                    foc_from_float(fQ));
            }
        }
    } else if (strncmp(args, "status", 6U) == 0) {
        foc_app_PrintStatus(&tFocApp.tMotor);
        return;
    } else {
        MLOGF(I, "usage: motor speed <e-turn/s> | current <d> <q> | "
              "voltage <d> <q> | align | stop | clear | status\r\n");
        return;
    }
    if (bStarted && eResult != FOC_RESULT_OK) {
        motor_Stop(&tFocApp.tMotor);
    }
    if (eResult != FOC_RESULT_OK) {
        MLOGF(W, "motor command rejected (%d)\r\n", (int)eResult);
    }
}

MODUS_SHELL_CMD(motor, foc_app_CmdMotor,
                "FOC Motor control: speed/current/voltage/align/stop/");
#endif

#if FOC_PORT_HAS_POSITION
MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tMotorCfg = {
        .tParams = {
            .chPolePairs = 7U,
            .wResistanceMilliohm = 500U,
            .wInductanceDMicroHenry = 1000U,
            .wInductanceQMicroHenry = 1000U,
        },
        .tLimits = {0},
        .tControl = {
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
            .wAdcCalibrationTimeoutSteps = 2000U,
            .wAlignSteps = 30000U,
            .chSpeedLoopDiv = 20U,
            .qAlignCurrent = FOC_SCALAR(0.10f),
        },
    },
    .tEncoderCfg = {
        .qSpeedFilterAlpha = FOC_SCALAR(0.25f),
        .wInvalidTimeoutUs = 5000U,
        .bDirectionInvert = false,
        .fnSensorInit = foc_port_PositionInit,
        .fnSensorRead = foc_port_PositionRead,
        .pSensorContext = NULL,
    }
    )
#else
MODUS_DECLARE_OBJECT(foc_app, FocApp,
    .tMotorCfg = {
        .tParams = {
            .chPolePairs = 7U,
            .wResistanceMilliohm = 500U,
            .wInductanceDMicroHenry = 1000U,
            .wInductanceQMicroHenry = 1000U,
        },
        .tLimits = {0},
        .tControl = {
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
            .wAdcCalibrationTimeoutSteps = 2000U,
            .wAlignSteps = 30000U,
            .chSpeedLoopDiv = 20U,
            .qAlignCurrent = FOC_SCALAR(0.10f),
        },
    },
    .tEncoderCfg = {0}
    )
#endif
