#include <stdio.h>
#include <string.h>

#include "motor.h"

static uint32_t s_wCriticalEnterCalls;
static uint32_t s_wCriticalExitCalls;

void perfc_test_CriticalEnter(void)
{
    s_wCriticalEnterCalls++;
}

void perfc_test_CriticalExit(void)
{
    s_wCriticalExitCalls++;
}

static void test_reset_critical_counts(void)
{
    s_wCriticalEnterCalls = 0U;
    s_wCriticalExitCalls = 0U;
}

static bool test_critical_was_balanced(void)
{
    return s_wCriticalEnterCalls > 0U &&
           s_wCriticalEnterCalls == s_wCriticalExitCalls;
}

typedef struct {
    uint32_t wInitCalls;
    uint32_t wContextTag;
    uint32_t wCalibrationBeginCalls;
    uint32_t wCalibrationStepCalls;
    uint32_t wCurrentSampleCalls;
    uint32_t wPositionReadCalls;
    uint32_t wSlowUpdateCalls;
    uint32_t wCaptureCalls;
    foc_scalar_t qElectricalSpeed;
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
    uint32_t wDutyCommitCalls;
    uint32_t wPwmEnableCalls;
    uint32_t wEmergencyStopCalls;
    bool bPwmEnabled;
    bool bPositionValid;
    bool bAdcCalibrationBusy;
    bool bZeroCaptured;
} motor_test_context_t;

static foc_result_t test_position_init(
    void *pContext,
    const motor_params_t *ptMotor,
    foc_scalar_t qHighFrequencyPeriod)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext == NULL || ptMotor == NULL ||
        qHighFrequencyPeriod <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptContext->wInitCalls++;
    return FOC_RESULT_OK;
}

static void test_position_reset(void *pContext)
{
    (void)pContext;
}

static int32_t test_position_slow_update(void *pContext)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wSlowUpdateCalls++;
    }
    return 0;
}

static foc_result_t test_position_read(
    void *pContext,
    motor_position_feedback_t *ptFeedback)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptContext != NULL) {
        ptContext->wPositionReadCalls++;
    }
    ptFeedback->tElectricalAngle = (foc_angle_t){0U};
    ptFeedback->qElectricalSpeed = ptContext == NULL
                                       ? FOC_ZERO
                                       : ptContext->qElectricalSpeed;
    ptFeedback->bValid = ptContext != NULL && ptContext->bPositionValid;
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnInit = test_position_init,
    .fnReset = test_position_reset,
    .fnSlowUpdate = test_position_slow_update,
    .fnObserve = NULL,
    .fnRead = test_position_read,
    .fnCaptureElectricalZero = NULL,
};

static foc_result_t test_position_capture_zero(void *pContext)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext == NULL) {
        return FOC_RESULT_NULL;
    }
    ptContext->wCaptureCalls++;
    if (!ptContext->bPositionValid) {
        return FOC_RESULT_SAFETY;
    }
    ptContext->bZeroCaptured = true;
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tCapturePositionOps = {
    .fnInit = test_position_init,
    .fnReset = test_position_reset,
    .fnSlowUpdate = test_position_slow_update,
    .fnObserve = NULL,
    .fnRead = test_position_read,
    .fnCaptureElectricalZero = test_position_capture_zero,
};

static void test_adc_begin(void *pContext,
                           foc_adc_calib_t *ptCalibration)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wCalibrationBeginCalls++;
    }
    (void)pContext;
    if (ptCalibration == NULL) {
        return;
    }
    memset(ptCalibration, 0, sizeof(*ptCalibration));
}

static foc_calibration_state_e test_adc_step(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wCalibrationStepCalls++;
    }
    (void)ptCalibration;
    if (ptContext != NULL && ptContext->bAdcCalibrationBusy) {
        return FOC_CALIBRATION_BUSY;
    }
    return FOC_CALIBRATION_COMPLETE;
}

static foc_result_t test_adc_sample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wCurrentSampleCalls++;
    }
    (void)ptCalibration;
    if (ptInput != NULL && ptContext != NULL) {
        ptInput->qIu = ptContext->qIu;
        ptInput->qIv = ptContext->qIv;
        ptInput->qIw = ptContext->qIw;
    }
    return FOC_RESULT_OK;
}

static const foc_adc_ops_t s_tAdcOps = {
    .fnCalibrationBegin = test_adc_begin,
    .fnCalibrationStep = test_adc_step,
    .fnCurrentSample = test_adc_sample,
};

static foc_result_t test_duty_commit(void *pContext,
                                     const foc_duty_abc_t *ptDuty)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wDutyCommitCalls++;
    }
    (void)ptDuty;
    return FOC_RESULT_OK;
}

static foc_result_t test_pwm_enable(void *pContext, bool bEnable)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wPwmEnableCalls++;
        ptContext->bPwmEnabled = bEnable;
    }
    (void)bEnable;
    return FOC_RESULT_OK;
}

static void test_emergency_stop(void *pContext)
{
    motor_test_context_t *ptContext = (motor_test_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wEmergencyStopCalls++;
        ptContext->bPwmEnabled = false;
    }
}

static const foc_pwm_ops_t s_tPwmOps = {
    .fnDutyCommit = test_duty_commit,
    .fnPwmEnable = test_pwm_enable,
    .fnEmergencyStop = test_emergency_stop,
};

static motor_cfg_t test_config(motor_test_context_t *ptContext)
{
    motor_cfg_t tConfig = {
        .tMotorParams = {
            .qResistance = FOC_SCALAR(0.1f),
            .qInductanceD = FOC_SCALAR(0.1f),
            .qInductanceQ = FOC_SCALAR(0.1f),
            .qFlux = FOC_SCALAR(0.1f),
            .wValidMask = MOTOR_PARAM_VALID_RS |
                          MOTOR_PARAM_VALID_LD |
                          MOTOR_PARAM_VALID_LQ |
                          MOTOR_PARAM_VALID_FLUX,
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
            .qHighFrequencyPeriod = FOC_SCALAR(0.00005f),
            .hwCalibrationTimeoutTicks = 2000U,
        },
        .ptAdcOps = &s_tAdcOps,
        .pAdcContext = ptContext,
        .ptPwmOps = &s_tPwmOps,
        .pPwmContext = ptContext,
        .tPosition = {
            .ptOps = &s_tPositionOps,
            .pContext = ptContext,
        },
    };

    return tConfig;
}

static motor_cfg_t test_cal_config(motor_test_context_t *ptContext)
{
    motor_cfg_t tConfig = test_config(ptContext);

    tConfig.tControlCfg.wPositionCalibrationTicks = 5U;
    tConfig.tPosition.ptOps = &s_tCapturePositionOps;
    return tConfig;
}

static int test_rejects_invalid_configuration(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {0};
    motor_cfg_t tConfig = test_config(&tContext);

    tConfig.tMotorParams.chPolePairs = 0U;
    return motor_Init(&tMotor, &tConfig) == FOC_RESULT_INVALID_ARGUMENT
               ? 0 : 1;
}

static int test_binds_two_contexts(void)
{
    motor_t tMotorA = {0};
    motor_t tMotorB = {0};
    motor_test_context_t tContextA = {0};
    motor_test_context_t tContextB = {0};
    motor_cfg_t tConfigA = test_config(&tContextA);
    motor_cfg_t tConfigB = test_config(&tContextB);

    if (motor_Init(&tMotorA, &tConfigA) != FOC_RESULT_OK ||
        motor_Init(&tMotorB, &tConfigB) != FOC_RESULT_OK) {
        return 1;
    }
    if (tContextA.wInitCalls != 1U || tContextB.wInitCalls != 1U) {
        return 1;
    }
    return tMotorA.pAdcContext == tMotorB.pAdcContext ? 1 : 0;
}

static int test_motor_lifecycle_and_safe_start(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_config(&tContext);
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_VOLTAGE,
        .tVoltageReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK ||
        tMotor.eLifecycle != MOTOR_STATE_INITIALIZING) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tContext.wCalibrationBeginCalls != 1U ||
        tContext.wCalibrationStepCalls != 0U ||
        tContext.wDutyCommitCalls != 0U ||
        tMotor.eLifecycle != MOTOR_STATE_CALIBRATING) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tContext.wCalibrationStepCalls != 1U ||
        tMotor.eLifecycle != MOTOR_STATE_IDLE ||
        tContext.bPwmEnabled) {
        return 1;
    }
    if (motor_Start(&tMotor, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_RUNNING ||
        tContext.wDutyCommitCalls != 1U ||
        tContext.wPwmEnableCalls != 1U ||
        !tContext.bPwmEnabled) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tContext.wCurrentSampleCalls != 1U ||
        tContext.wPositionReadCalls != 1U ||
        tContext.wDutyCommitCalls != 2U) {
        return 1;
    }
    motor_Stop(&tMotor);
    return (tContext.wEmergencyStopCalls == 1U &&
            tMotor.eLifecycle == MOTOR_STATE_IDLE &&
            !tContext.bPwmEnabled) ? 0 : 1;
}

static int test_motor_speed_loop_mode_guard(void)
{
    motor_t tSpeedMotor = {0};
    motor_t tCurrentMotor = {0};
    motor_test_context_t tSpeedContext = {
        .bPositionValid = true,
        .qElectricalSpeed = FOC_SCALAR(80.0f / 7.0f),
    };
    motor_test_context_t tCurrentContext = {
        .bPositionValid = true,
    };
    motor_cfg_t tSpeedConfig = test_config(&tSpeedContext);
    motor_cfg_t tCurrentConfig = test_config(&tCurrentContext);
    foc_core_command_t tSpeedCommand = {
        .eMode = FOC_MODE_SPEED,
        .qSpeedReference = FOC_SCALAR(100.0f),
    };
    foc_core_command_t tCurrentCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };
    foc_scalar_t qCurrentBefore = FOC_ZERO;

    if (motor_Init(&tSpeedMotor, &tSpeedConfig) != FOC_RESULT_OK ||
        motor_Init(&tCurrentMotor, &tCurrentConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tSpeedMotor);
    motor_HighFrequencyStep(&tSpeedMotor);
    motor_HighFrequencyStep(&tCurrentMotor);
    motor_HighFrequencyStep(&tCurrentMotor);
    if (motor_Start(&tSpeedMotor, &tSpeedCommand) != FOC_RESULT_OK ||
        motor_Start(&tCurrentMotor, &tCurrentCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tSpeedMotor);
    motor_HighFrequencyStep(&tSpeedMotor);
    motor_HighFrequencyStep(&tCurrentMotor);
    motor_HighFrequencyStep(&tCurrentMotor);
    qCurrentBefore = tCurrentMotor.tCommand.tCurrentReference.qQ;
    motor_ClockStep(&tSpeedMotor);
    motor_ClockStep(&tCurrentMotor);
    if (tSpeedMotor.tCommand.tCurrentReference.qQ == FOC_ZERO) {
        return 1;
    }
    return tCurrentMotor.tCommand.tCurrentReference.qQ == qCurrentBefore
               ? 0 : 1;
}

static int test_speed_loop_respects_command_iq_limit(void)
{
    motor_t tPositiveMotor = {0};
    motor_t tNegativeMotor = {0};
    motor_test_context_t tPositiveContext = {
        .bPositionValid = true,
    };
    motor_test_context_t tNegativeContext = {
        .bPositionValid = true,
    };
    motor_cfg_t tPositiveConfig = test_config(&tPositiveContext);
    motor_cfg_t tNegativeConfig = test_config(&tNegativeContext);
    foc_core_command_t tPositiveCommand = {
        .eMode = FOC_MODE_SPEED,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.03f)},
        .qSpeedReference = FOC_SCALAR(100.0f),
    };
    foc_core_command_t tNegativeCommand = {
        .eMode = FOC_MODE_SPEED,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.03f)},
        .qSpeedReference = FOC_SCALAR(-100.0f),
    };

    if (motor_Init(&tPositiveMotor, &tPositiveConfig) != FOC_RESULT_OK ||
        motor_Init(&tNegativeMotor, &tNegativeConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tPositiveMotor);
    motor_HighFrequencyStep(&tPositiveMotor);
    motor_HighFrequencyStep(&tNegativeMotor);
    motor_HighFrequencyStep(&tNegativeMotor);
    if (motor_Start(&tPositiveMotor, &tPositiveCommand) != FOC_RESULT_OK ||
        motor_Start(&tNegativeMotor, &tNegativeCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tPositiveMotor);
    motor_HighFrequencyStep(&tNegativeMotor);
    motor_ClockStep(&tPositiveMotor);
    motor_ClockStep(&tNegativeMotor);
    return (tPositiveMotor.tCommand.tCurrentReference.qQ ==
                FOC_SCALAR(0.03f) &&
            tNegativeMotor.tCommand.tCurrentReference.qQ ==
                FOC_SCALAR(-0.03f)) ? 0 : 1;
}

static int test_motor_api_owns_command_synchronization(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_config(&tContext);
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_SPEED,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.03f)},
        .qSpeedReference = FOC_SCALAR(50.0f),
    };
    motor_feedback_t tFeedback = {0};
    motor_status_t tStatus = {0};

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);

    test_reset_critical_counts();
    if (motor_Start(&tMotor, &tCommand) != FOC_RESULT_OK ||
        !test_critical_was_balanced()) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);

    test_reset_critical_counts();
    if (motor_SetSpeedReference(&tMotor, FOC_SCALAR(60.0f)) !=
            FOC_RESULT_OK || !test_critical_was_balanced()) {
        return 1;
    }
    test_reset_critical_counts();
    if (motor_GetFeedback(&tMotor, &tFeedback) != FOC_RESULT_OK ||
        !test_critical_was_balanced()) {
        return 1;
    }
    test_reset_critical_counts();
    if (motor_GetStatus(&tMotor, &tStatus) != FOC_RESULT_OK ||
        !test_critical_was_balanced()) {
        return 1;
    }
    test_reset_critical_counts();
    motor_Stop(&tMotor);
    return test_critical_was_balanced() ? 0 : 1;
}

static int test_motor_background_and_query_api(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_config(&tContext);
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_SCALAR(0.10f), FOC_ZERO},
    };
    motor_feedback_t tFeedback = {0};
    motor_status_t tStatus = {0};

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_BackgroundStep(&tMotor);
    if (tContext.wSlowUpdateCalls != 1U) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_Start(&tMotor, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_SetCurrentReference(&tMotor, FOC_SCALAR(0.10f),
                                  FOC_SCALAR(0.02f)) != FOC_RESULT_OK ||
        motor_SetSpeedReference(&tMotor, FOC_SCALAR(10.0f)) !=
            FOC_RESULT_INVALID_ARGUMENT ||
        motor_CaptureElectricalZero(&tMotor) != FOC_RESULT_DISABLED ||
        motor_GetFeedback(&tMotor, &tFeedback) != FOC_RESULT_OK ||
        motor_GetStatus(&tMotor, &tStatus) != FOC_RESULT_OK) {
        return 1;
    }
    return tStatus.eLifecycle == MOTOR_STATE_RUNNING &&
                   tFeedback.tPosition.bValid
               ? 0
               : 1;
}

static int test_position_cal_requires_capture_provider(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_config(&tContext);

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_IDLE) {
        return 1;
    }
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_DISABLED) {
        return 1;
    }
    return (tMotor.eLifecycle == MOTOR_STATE_IDLE &&
            tContext.wEmergencyStopCalls == 0U &&
            !tContext.bPwmEnabled) ? 0 : 1;
}

static int test_position_cal_rejects_bad_state_and_args(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_cal_config(&tContext);
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_RequestPositionCalibration(&tMotor, FOC_ZERO) !=
            FOC_RESULT_INVALID_ARGUMENT ||
        motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(1.5f)) !=
            FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    if (motor_Start(&tMotor, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_RUNNING ||
        motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f)) !=
            FOC_RESULT_BUSY) {
        return 1;
    }
    motor_Stop(&tMotor);
    return tMotor.eLifecycle == MOTOR_STATE_IDLE ? 0 : 1;
}

static int test_position_cal_align_then_capture(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_cal_config(&tContext);
    uint32_t wIdx = 0U;

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_POSITION_CAL ||
        !tContext.bPwmEnabled ||
        tContext.wPwmEnableCalls != 1U ||
        tMotor.tCommand.eMode != FOC_MODE_CURRENT ||
        tMotor.tCommand.tCurrentReference.qD != FOC_SCALAR(0.10f) ||
        tMotor.tCommand.tCurrentReference.qQ != FOC_ZERO) {
        return 1;
    }
    for (wIdx = 0U; wIdx < 4U; wIdx++) {
        motor_HighFrequencyStep(&tMotor);
    }
    motor_BackgroundStep(&tMotor);
    if (tContext.wCaptureCalls != 0U ||
        tMotor.eLifecycle != MOTOR_STATE_POSITION_CAL) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_BackgroundStep(&tMotor);
    if (tContext.wCaptureCalls != 1U ||
        !tContext.bZeroCaptured ||
        tMotor.eLifecycle != MOTOR_STATE_IDLE ||
        tContext.bPwmEnabled ||
        tContext.wEmergencyStopCalls != 1U ||
        tMotor.wFaults != 0U) {
        return 1;
    }
    motor_BackgroundStep(&tMotor);
    return tContext.wCaptureCalls == 1U ? 0 : 1;
}

static int test_position_cal_adc_before_align(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_cal_config(&tContext);

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK ||
        tMotor.eLifecycle != MOTOR_STATE_INITIALIZING) {
        return 1;
    }
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK) {
        return 1;
    }
    if (tMotor.eLifecycle != MOTOR_STATE_CALIBRATING ||
        tContext.wCalibrationBeginCalls != 1U) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    return (tMotor.eLifecycle == MOTOR_STATE_POSITION_CAL &&
            tContext.bPwmEnabled &&
            tContext.wCalibrationStepCalls == 1U) ? 0 : 1;
}

static int test_position_cal_invalid_position_fault(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = false};
    motor_cfg_t tConfig = test_cal_config(&tContext);
    uint32_t wIdx = 0U;

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    for (wIdx = 0U; wIdx < 5U; wIdx++) {
        motor_HighFrequencyStep(&tMotor);
    }
    motor_BackgroundStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_FAULT ||
        (tMotor.wFaults & MOTOR_FAULT_POSITION_CAL) == 0U ||
        tContext.bPwmEnabled ||
        tContext.wEmergencyStopCalls == 0U ||
        tContext.bZeroCaptured) {
        return 1;
    }
    if (motor_ClearFault(&tMotor) != FOC_RESULT_OK ||
        tMotor.eLifecycle != MOTOR_STATE_IDLE ||
        tMotor.wFaults != 0U) {
        return 1;
    }
    tContext.bPositionValid = true;
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    for (wIdx = 0U; wIdx < 5U; wIdx++) {
        motor_HighFrequencyStep(&tMotor);
    }
    motor_BackgroundStep(&tMotor);
    return (tContext.bZeroCaptured &&
            tMotor.eLifecycle == MOTOR_STATE_IDLE &&
            tMotor.wFaults == 0U) ? 0 : 1;
}

static int test_position_cal_adc_timeout_fault(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {
        .bPositionValid = true,
        .bAdcCalibrationBusy = true,
    };
    motor_cfg_t tConfig = test_cal_config(&tContext);
    uint32_t wIdx = 0U;

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK ||
        tMotor.eLifecycle != MOTOR_STATE_CALIBRATING) {
        return 1;
    }
    for (wIdx = 0U; wIdx <= 2000U; wIdx++) {
        motor_HighFrequencyStep(&tMotor);
    }
    if (tMotor.eLifecycle != MOTOR_STATE_FAULT ||
        (tMotor.wFaults & MOTOR_FAULT_CALIBRATION_TIMEOUT) == 0U ||
        tContext.wPwmEnableCalls != 0U) {
        return 1;
    }
    return motor_ClearFault(&tMotor) == FOC_RESULT_OK &&
                   tMotor.eLifecycle == MOTOR_STATE_IDLE
               ? 0
               : 1;
}

static int test_position_cal_stop_cancels(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {.bPositionValid = true};
    motor_cfg_t tConfig = test_cal_config(&tContext);

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_RequestPositionCalibration(&tMotor, FOC_SCALAR(0.10f))
            != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.eLifecycle != MOTOR_STATE_POSITION_CAL) {
        return 1;
    }
    motor_Stop(&tMotor);
    motor_BackgroundStep(&tMotor);
    return (tMotor.eLifecycle == MOTOR_STATE_IDLE &&
            !tContext.bPwmEnabled &&
            tContext.wEmergencyStopCalls == 1U &&
            tContext.wCaptureCalls == 0U &&
            !tContext.bZeroCaptured) ? 0 : 1;
}

static int test_motor_phase_snapshot(void)
{
    motor_t tMotor = {0};
    motor_test_context_t tContext = {
        .bPositionValid = true,
        .qIu = FOC_SCALAR(0.050f),
        .qIv = FOC_SCALAR(-0.030f),
        .qIw = FOC_SCALAR(0.020f),
    };
    motor_cfg_t tConfig = test_config(&tContext);
    foc_core_command_t tCommand = {
        .eMode = FOC_MODE_CURRENT,
        .tCurrentReference = {FOC_ZERO, FOC_SCALAR(0.05f)},
    };

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_Start(&tMotor, &tCommand) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    return (tMotor.qIuLatest == FOC_SCALAR(0.050f) &&
            tMotor.qIvLatest == FOC_SCALAR(-0.030f) &&
            tMotor.qIwLatest == FOC_SCALAR(0.020f)) ? 0 : 1;
}

int main(void)
{
    int nFailures = 0;
    int nSub = test_rejects_invalid_configuration();

    printf("  invalid_config: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_binds_two_contexts();
    printf("  two_contexts: %s\n", nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_motor_lifecycle_and_safe_start();
    printf("  lifecycle_safe_start: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_motor_speed_loop_mode_guard();
    printf("  speed_loop_mode_guard: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_speed_loop_respects_command_iq_limit();
    printf("  speed_loop_iq_limit: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_motor_api_owns_command_synchronization();
    printf("  motor_api_sync: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_motor_background_and_query_api();
    printf("  background_query_api: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_requires_capture_provider();
    printf("  position_cal_no_capture: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_rejects_bad_state_and_args();
    printf("  position_cal_state_gating: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_align_then_capture();
    printf("  position_cal_align_capture: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_adc_before_align();
    printf("  position_cal_adc_first: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_invalid_position_fault();
    printf("  position_cal_invalid_fault: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_adc_timeout_fault();
    printf("  position_cal_adc_timeout: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_position_cal_stop_cancels();
    printf("  position_cal_stop_cancel: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    nSub = test_motor_phase_snapshot();
    printf("  phase_snapshot: %s\n",
           nSub == 0 ? "PASS" : "FAIL");
    nFailures += nSub;
    printf("motor contract: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
