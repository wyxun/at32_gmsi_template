#include <stdio.h>

#include "motor.h"

static uint32_t s_wCriticalEnterCalls;
static uint32_t s_wCriticalExitCalls;
static int64_t s_lNowMs;

void perfc_test_CriticalEnter(void)
{
    s_wCriticalEnterCalls++;
}

void perfc_test_CriticalExit(void)
{
    s_wCriticalExitCalls++;
}

int64_t get_system_ms(void)
{
    return s_lNowMs;
}

typedef struct {
    uint32_t wCalibrationSteps;
    uint32_t wPolls;
    uint32_t wReads;
    uint32_t wCaptures;
    uint32_t wDutyCommits;
    uint32_t wEnableCalls;
    uint32_t wEmergencyStops;
    uint32_t wCalibrationLimit;
    uint32_t wAlignLimit;
    foc_scalar_t qSpeed;
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
    bool bPwmEnabled;
    bool bFeedbackValid;
    bool bCalibrationFails;
    bool bPollFails;
} motor_fake_t;

static foc_result_t test_position_init(void *pContext,
                                       const motor_params_t *ptMotor,
                                       foc_scalar_t qHighFrequencyPeriod,
                                       const foc_encoder_params_t *ptEncoder)
{
    (void)pContext;
    return ptMotor == NULL || ptEncoder == NULL ||
                   qHighFrequencyPeriod <= FOC_ZERO
               ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

static void test_position_reset(void *pContext)
{
    (void)pContext;
}

static foc_result_t test_position_poll(void *pContext)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wPolls++;
    return ptFake->bPollFails ? FOC_RESULT_SAFETY : FOC_RESULT_OK;
}

static foc_result_t test_position_read(
    void *pContext, motor_position_feedback_t *ptFeedback)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wReads++;
    ptFeedback->tElectricalAngle = (foc_angle_t){0U};
    ptFeedback->qElectricalSpeed = ptFake->qSpeed;
    ptFeedback->tMechanicalAngle = (foc_angle_t){0U};
    ptFeedback->qMechanicalSpeed = FOC_ZERO;
    ptFeedback->bValid = ptFake->bFeedbackValid;
    return FOC_RESULT_OK;
}

static foc_result_t test_position_capture(void *pContext)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wCaptures++;
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnInit = test_position_init,
    .fnReset = test_position_reset,
    .fnPoll = test_position_poll,
    .fnReadFeedback = test_position_read,
    .fnCaptureZero = test_position_capture,
};

static void test_adc_begin(void *pContext,
                           foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    if (ptCalibration != NULL) {
        *ptCalibration = (foc_adc_calib_t){0};
    }
}

static foc_calibration_state_e test_adc_step(
    void *pContext, foc_adc_calib_t *ptCalibration)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    (void)ptCalibration;
    if (ptFake == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    ptFake->wCalibrationSteps++;
    if (ptFake->bCalibrationFails) {
        return FOC_CALIBRATION_FAILED;
    }
    return ptFake->wCalibrationSteps >= ptFake->wCalibrationLimit
               ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_BUSY;
}

static foc_result_t test_adc_sample(
    void *pContext, const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    (void)ptCalibration;
    if (ptFake == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    ptInput->qIu = ptFake->qIu;
    ptInput->qIv = ptFake->qIv;
    ptInput->qIw = ptFake->qIw;
    ptInput->bAngleValid = true;
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
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wDutyCommits++;
    return FOC_RESULT_OK;
}

static foc_result_t test_pwm_enable(void *pContext, bool bEnable)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wEnableCalls++;
    ptFake->bPwmEnabled = bEnable;
    return FOC_RESULT_OK;
}

static void test_emergency_stop(void *pContext)
{
    motor_fake_t *ptFake = (motor_fake_t *)pContext;

    if (ptFake != NULL) {
        ptFake->wEmergencyStops++;
        ptFake->bPwmEnabled = false;
    }
}

static const foc_pwm_ops_t s_tPwmOps = {
    .fnDutyCommit = test_duty_commit,
    .fnPwmEnable = test_pwm_enable,
    .fnEmergencyStop = test_emergency_stop,
};

static foc_pid_params_t test_pi_params(float fOutput)
{
    foc_pid_params_t tParams = {
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .qOutputMinimum = FOC_SCALAR(-fOutput),
        .qOutputMaximum = FOC_SCALAR(fOutput),
        .qIntegratorMinimum = FOC_SCALAR(-fOutput),
        .qIntegratorMaximum = FOC_SCALAR(fOutput),
    };

    return tParams;
}

static motor_cfg_t test_config(motor_fake_t *ptFake)
{
    motor_cfg_t tConfig = {0};

    tConfig.tMotorParams.chPolePairs = 7U;
    tConfig.tEncoderParams.qSpeedFilterAlpha = FOC_SCALAR(0.25f);
    tConfig.tEncoderParams.hwInvalidTimeout = 100U;
    tConfig.tControlCfg.tCurrentPiParams = test_pi_params(0.55f);
    tConfig.tControlCfg.tSpeedPiParams = test_pi_params(0.10f);
    tConfig.tControlCfg.qHighFrequencyPeriod = FOC_SCALAR(0.00005f);
    tConfig.tControlCfg.qPositionCalibrationCurrent =
        FOC_SCALAR(0.10f);
    tConfig.tControlCfg.qSpeedIqLimit = FOC_SCALAR(0.10f);
    tConfig.tControlCfg.hwCalibrationTimeoutTicks = 2000U;
    tConfig.tControlCfg.wPositionCalibrationTicks = 5U;
    tConfig.ptAdcOps = &s_tAdcOps;
    tConfig.pAdcContext = ptFake;
    tConfig.ptPwmOps = &s_tPwmOps;
    tConfig.pPwmContext = ptFake;
    tConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tPosition.pContext = ptFake;
    return tConfig;
}

static void test_init_to_idle(motor_t *ptMotor, motor_fake_t *ptFake)
{
    motor_cfg_t tConfig = test_config(ptFake);

    ptFake->wCalibrationLimit = 1U;
    (void)motor_Init(ptMotor, &tConfig);
    motor_HighFrequencyStep(ptMotor);
    motor_HighFrequencyStep(ptMotor);
}

static int test_adc_cal_never_enables_pwm(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 100U};
    motor_cfg_t tConfig = test_config(&tFake);

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    return tMotor.tLifecycle.eLifecycle == MOTOR_STATE_ADC_CAL &&
                   !tFake.bPwmEnabled && tFake.wEnableCalls == 0U
               ? 0 : 1;
}

static int test_start_during_adc_cal_is_busy(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 2U};
    motor_cfg_t tConfig = test_config(&tFake);

    if (motor_Init(&tMotor, &tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    if (motor_Start(&tMotor, FOC_MODE_CURRENT) != FOC_RESULT_BUSY) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.tLifecycle.eLifecycle != MOTOR_STATE_ADC_CAL ||
        motor_Start(&tMotor, FOC_MODE_CURRENT) != FOC_RESULT_BUSY) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    return tMotor.tLifecycle.eLifecycle == MOTOR_STATE_IDLE &&
                   !tFake.bPwmEnabled
               ? 0 : 1;
}

static int test_typed_references_and_start(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 1U,
                          .bFeedbackValid = true};
    motor_status_t tStatus = {0};

    test_init_to_idle(&tMotor, &tFake);
    if (motor_SetVoltageReference(&tMotor, FOC_ZERO,
                                  FOC_SCALAR(0.05f)) != FOC_RESULT_OK ||
        motor_SetCurrentReference(&tMotor, FOC_ZERO,
                                  FOC_SCALAR(0.05f)) != FOC_RESULT_OK ||
        motor_SetSpeedReference(&tMotor, FOC_SCALAR(20.0f)) !=
            FOC_RESULT_OK ||
        motor_SetPositionReference(&tMotor, (foc_angle_t){1U}) !=
            FOC_RESULT_DISABLED ||
        motor_Start(&tMotor, FOC_MODE_CURRENT) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (motor_GetStatus(&tMotor, &tStatus) != FOC_RESULT_OK ||
        tStatus.eMode != FOC_MODE_CURRENT || !tFake.bPwmEnabled) {
        return 1;
    }
    return motor_SetVoltageReference(&tMotor, FOC_ZERO, FOC_ZERO) ==
                   FOC_RESULT_INVALID_ARGUMENT
               ? 0 : 1;
}

static int test_speed_pi_runs_once_per_twenty_ticks(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 1U,
                          .bFeedbackValid = true,
                          .qSpeed = FOC_ZERO};
    foc_scalar_t qIntegrator = FOC_ZERO;
    uint32_t wIndex = 0U;

    test_init_to_idle(&tMotor, &tFake);
    if (motor_SetSpeedReference(&tMotor, FOC_SCALAR(0.20f)) !=
            FOC_RESULT_OK ||
        motor_Start(&tMotor, FOC_MODE_SPEED) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    qIntegrator = tMotor.tControl.tSpeedPi.qIntegrator;
    for (wIndex = 0U; wIndex < 19U; wIndex++) {
        motor_HighFrequencyStep(&tMotor);
    }
    if (tMotor.tControl.tSpeedPi.qIntegrator != qIntegrator) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.tControl.tSpeedPi.qIntegrator == qIntegrator) {
        return 1;
    }
    return 0;
}

static int test_alignment_requires_adc_and_exits_safe(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 1U,
                          .bFeedbackValid = true};
    uint32_t wIndex = 0U;

    test_init_to_idle(&tMotor, &tFake);
    if (motor_RequestPositionCalibration(&tMotor) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotor);
    if (tMotor.tLifecycle.eLifecycle != MOTOR_STATE_ALIGN ||
        !tFake.bPwmEnabled ||
        tMotor.tControl.tCurrentReference.qD != FOC_SCALAR(0.10f) ||
        tMotor.tControl.tCurrentReference.qQ != FOC_ZERO ||
        tMotor.tControl.tCore.tVoltage.qD <= FOC_ZERO ||
        tMotor.tControl.tCore.tVoltage.qQ != FOC_ZERO) {
        return 1;
    }
    for (wIndex = 0U; wIndex < 5U; wIndex++) {
        motor_HighFrequencyStep(&tMotor);
    }
    motor_BackgroundStep(&tMotor);
    return tMotor.tLifecycle.eLifecycle == MOTOR_STATE_IDLE &&
                   !tFake.bPwmEnabled && tFake.wCaptures == 1U
               ? 0 : 1;
}

static int test_two_instances_have_independent_speed_pi(void)
{
    motor_t tMotorA = {0};
    motor_t tMotorB = {0};
    motor_fake_t tFakeA = {.wCalibrationLimit = 1U,
                           .bFeedbackValid = true};
    motor_fake_t tFakeB = {.wCalibrationLimit = 1U,
                           .bFeedbackValid = true};

    test_init_to_idle(&tMotorA, &tFakeA);
    test_init_to_idle(&tMotorB, &tFakeB);
    if (motor_SetSpeedReference(&tMotorA, FOC_SCALAR(0.20f)) !=
            FOC_RESULT_OK ||
        motor_Start(&tMotorA, FOC_MODE_SPEED) != FOC_RESULT_OK ||
        motor_SetSpeedReference(&tMotorB, FOC_SCALAR(-0.20f)) !=
            FOC_RESULT_OK ||
        motor_Start(&tMotorB, FOC_MODE_SPEED) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&tMotorA);
    motor_HighFrequencyStep(&tMotorB);
    if (!(tMotorA.tControl.tSpeedPi.qIntegrator !=
                   tMotorB.tControl.tSpeedPi.qIntegrator &&
                   tMotorA.tControl.tCurrentReference.qQ !=
                       tMotorB.tControl.tCurrentReference.qQ)) {
        return 1;
    }
    return 0;
}

static int test_foreground_poll_rate_and_backoff(void)
{
    motor_t tMotor = {0};
    motor_fake_t tFake = {.wCalibrationLimit = 1U,
                          .bFeedbackValid = true,
                          .bPollFails = false};

    s_lNowMs = 0;
    test_init_to_idle(&tMotor, &tFake);
    motor_BackgroundStep(&tMotor);
    motor_BackgroundStep(&tMotor);
    if (tFake.wPolls != 1U) {
        return 1;
    }
    s_lNowMs = 1;
    motor_BackgroundStep(&tMotor);
    if (tFake.wPolls != 2U) {
        return 1;
    }
    tFake.bPollFails = true;
    s_lNowMs = 2;
    motor_BackgroundStep(&tMotor);
    s_lNowMs = 3;
    motor_BackgroundStep(&tMotor);
    if (tFake.wPolls != 3U ||
        tMotor.tSchedule.hwConsecutivePollFails != 1U) {
        return 1;
    }
    s_lNowMs = 102;
    motor_BackgroundStep(&tMotor);
    return tFake.wPolls == 4U &&
           tMotor.tSchedule.hwConsecutivePollFails == 2U
               ? 0 : 1;
}

int main(void)
{
    int nFailures = 0;
    int nResult = 0;

    nResult = test_adc_cal_never_enables_pwm();
    printf("  adc_cal_safe: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_start_during_adc_cal_is_busy();
    printf("  adc_start_guard: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_typed_references_and_start();
    printf("  typed_start: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_speed_pi_runs_once_per_twenty_ticks();
    printf("  speed_divider: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_alignment_requires_adc_and_exits_safe();
    printf("  align_safe: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_two_instances_have_independent_speed_pi();
    printf("  two_instances: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_foreground_poll_rate_and_backoff();
    printf("  foreground_poll: %s\n",
           nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    printf("motor framework: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
