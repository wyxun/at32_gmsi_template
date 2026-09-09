#include <stdio.h>

#include "motor.h"

void perfc_test_CriticalEnter(void)
{
}

void perfc_test_CriticalExit(void)
{
}

int64_t get_system_ms(void)
{
    return 0;
}

typedef struct {
    uint32_t wPolls;
    uint32_t wReads;
    uint32_t wDutyCommits;
    uint32_t wEnableCalls;
    uint32_t wEmergencyStops;
    bool bPwmEnabled;
} contract_context_t;

static foc_result_t contract_position_init(
    void *pContext,
    const motor_params_t *ptMotor,
    foc_scalar_t qHighFrequencyPeriod,
    const foc_encoder_params_t *ptEncoder)
{
    (void)pContext;
    return ptMotor == NULL || ptEncoder == NULL ||
                   qHighFrequencyPeriod <= FOC_ZERO
               ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

static void contract_position_reset(void *pContext)
{
    (void)pContext;
}

static foc_result_t contract_position_poll(void *pContext)
{
    contract_context_t *ptContext = (contract_context_t *)pContext;

    if (ptContext == NULL) {
        return FOC_RESULT_NULL;
    }
    ptContext->wPolls++;
    return FOC_RESULT_OK;
}

static foc_result_t contract_position_read(
    void *pContext, motor_position_feedback_t *ptFeedback)
{
    contract_context_t *ptContext = (contract_context_t *)pContext;

    if (ptContext == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    ptContext->wReads++;
    ptFeedback->tElectricalAngle = (foc_angle_t){0U};
    ptFeedback->qElectricalSpeed = FOC_ZERO;
    ptFeedback->bValid = true;
    return FOC_RESULT_OK;
}

static foc_result_t contract_position_capture(void *pContext)
{
    return pContext == NULL ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnInit = contract_position_init,
    .fnReset = contract_position_reset,
    .fnPoll = contract_position_poll,
    .fnReadFeedback = contract_position_read,
    .fnCaptureZero = contract_position_capture,
};

static void contract_adc_begin(void *pContext,
                               foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    (void)ptCalibration;
}

static foc_calibration_state_e contract_adc_step(
    void *pContext, foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    (void)ptCalibration;
    return FOC_CALIBRATION_COMPLETE;
}

static foc_result_t contract_adc_sample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    (void)pContext;
    (void)ptCalibration;
    if (ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    ptInput->bAngleValid = true;
    return FOC_RESULT_OK;
}

static const foc_adc_ops_t s_tAdcOps = {
    .fnCalibrationBegin = contract_adc_begin,
    .fnCalibrationStep = contract_adc_step,
    .fnCurrentSample = contract_adc_sample,
};

static foc_result_t contract_duty_commit(
    void *pContext, const foc_duty_abc_t *ptDuty)
{
    contract_context_t *ptContext = (contract_context_t *)pContext;

    if (ptContext == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    ptContext->wDutyCommits++;
    return FOC_RESULT_OK;
}

static foc_result_t contract_pwm_enable(void *pContext, bool bEnable)
{
    contract_context_t *ptContext = (contract_context_t *)pContext;

    if (ptContext == NULL) {
        return FOC_RESULT_NULL;
    }
    ptContext->wEnableCalls++;
    ptContext->bPwmEnabled = bEnable;
    return FOC_RESULT_OK;
}

static void contract_emergency_stop(void *pContext)
{
    contract_context_t *ptContext = (contract_context_t *)pContext;

    if (ptContext != NULL) {
        ptContext->wEmergencyStops++;
        ptContext->bPwmEnabled = false;
    }
}

static const foc_pwm_ops_t s_tPwmOps = {
    .fnDutyCommit = contract_duty_commit,
    .fnPwmEnable = contract_pwm_enable,
    .fnEmergencyStop = contract_emergency_stop,
};

static motor_cfg_t contract_config(contract_context_t *ptContext)
{
    motor_cfg_t tConfig = {0};

    tConfig.tMotorParams.chPolePairs = 7U;
    tConfig.tEncoderParams.qSpeedFilterAlpha = FOC_SCALAR(0.25f);
    tConfig.tEncoderParams.hwInvalidTimeout = 100U;
    tConfig.tControlCfg.tCurrentPiParams = (foc_pid_params_t){
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .qOutputMinimum = FOC_SCALAR(-0.55f),
        .qOutputMaximum = FOC_SCALAR(0.55f),
        .qIntegratorMinimum = FOC_SCALAR(-0.50f),
        .qIntegratorMaximum = FOC_SCALAR(0.50f),
    };
    tConfig.tControlCfg.tSpeedPiParams = tConfig.tControlCfg.
        tCurrentPiParams;
    tConfig.tControlCfg.qHighFrequencyPeriod = FOC_SCALAR(0.00005f);
    tConfig.tControlCfg.hwCalibrationTimeoutTicks = 2000U;
    tConfig.tControlCfg.wPositionCalibrationTicks = 30000U;
    tConfig.tControlCfg.qPositionCalibrationCurrent = FOC_SCALAR(0.10f);
    tConfig.tControlCfg.qSpeedIqLimit = FOC_SCALAR(0.10f);
    tConfig.ptAdcOps = &s_tAdcOps;
    tConfig.pAdcContext = ptContext;
    tConfig.ptPwmOps = &s_tPwmOps;
    tConfig.pPwmContext = ptContext;
    tConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tPosition.pContext = ptContext;
    return tConfig;
}

static int test_typed_motor_api(void)
{
    motor_t tMotor = {0};
    contract_context_t tContext = {0};
    motor_cfg_t tConfig = contract_config(&tContext);
    motor_status_t tStatus = {0};

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    if (motor_Start(&tMotor, FOC_MODE_MAX) !=
        FOC_RESULT_INVALID_ARGUMENT) {
        return 1;
    }
    if (motor_Start(&tMotor, FOC_MODE_POSITION) !=
        FOC_RESULT_DISABLED) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (motor_SetCurrentReference(&tMotor, FOC_ZERO,
                                  FOC_SCALAR(0.05f)) != FOC_RESULT_OK ||
        motor_Start(&tMotor, FOC_MODE_CURRENT) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (motor_GetStatus(&tMotor, &tStatus) != FOC_RESULT_OK ||
        tStatus.eMode != FOC_MODE_CURRENT || !tContext.bPwmEnabled) {
        return 1;
    }
    motor_Stop(&tMotor);
    return 0;
}

int main(void)
{
    int nFailures = test_typed_motor_api();

    printf("framework contract: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
