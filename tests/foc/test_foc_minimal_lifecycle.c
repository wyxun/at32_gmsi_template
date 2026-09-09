#include <stdio.h>
#include <string.h>

#include "foc_app.h"
#include "foc_port.h"

typedef struct {
    uint32_t wCalibrationSteps;
    uint32_t wPolls;
    uint32_t wReads;
    uint32_t wDutyCommits;
    uint32_t wEnableCalls;
    uint32_t wEmergencyStops;
    bool bPwmEnabled;
    bool bFeedbackValid;
} app_fake_t;

static app_fake_t s_tFake;
static foc_app_t s_tApp;
static int64_t s_lNowMs;

int64_t get_system_ms(void)
{
    return s_lNowMs;
}

static foc_result_t app_position_init(void *pContext,
                                      const motor_params_t *ptMotor,
                                      foc_scalar_t qHighFrequencyPeriod,
                                      const foc_encoder_params_t *ptEncoder)
{
    (void)pContext;
    return ptMotor == NULL || ptEncoder == NULL ||
                   qHighFrequencyPeriod <= FOC_ZERO
               ? FOC_RESULT_NULL : FOC_RESULT_OK;
}

static void app_position_reset(void *pContext)
{
    (void)pContext;
}

static foc_result_t app_position_poll(void *pContext)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    if (ptFake == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wPolls++;
    return FOC_RESULT_OK;
}

static foc_result_t app_position_read(
    void *pContext, motor_position_feedback_t *ptFeedback)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    if (ptFake == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wReads++;
    ptFeedback->tElectricalAngle = (foc_angle_t){0U};
    ptFeedback->qElectricalSpeed = FOC_ZERO;
    ptFeedback->bValid = ptFake->bFeedbackValid;
    return FOC_RESULT_OK;
}

static const motor_position_ops_t s_tPositionOps = {
    .fnInit = app_position_init,
    .fnReset = app_position_reset,
    .fnPoll = app_position_poll,
    .fnReadFeedback = app_position_read,
    .fnCaptureZero = NULL,
};

static void app_adc_begin(void *pContext,
                          foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    if (ptCalibration != NULL) {
        *ptCalibration = (foc_adc_calib_t){0};
    }
}

static foc_calibration_state_e app_adc_step(
    void *pContext, foc_adc_calib_t *ptCalibration)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    (void)ptCalibration;
    if (ptFake == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    ptFake->wCalibrationSteps++;
    return FOC_CALIBRATION_COMPLETE;
}

static foc_result_t app_adc_sample(
    void *pContext, const foc_adc_calib_t *ptCalibration,
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
    .fnCalibrationBegin = app_adc_begin,
    .fnCalibrationStep = app_adc_step,
    .fnCurrentSample = app_adc_sample,
};

static foc_result_t app_duty_commit(void *pContext,
                                    const foc_duty_abc_t *ptDuty)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    if (ptFake == NULL || ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wDutyCommits++;
    return FOC_RESULT_OK;
}

static foc_result_t app_pwm_enable(void *pContext, bool bEnable)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    if (ptFake == NULL) {
        return FOC_RESULT_NULL;
    }
    ptFake->wEnableCalls++;
    ptFake->bPwmEnabled = bEnable;
    return FOC_RESULT_OK;
}

static void app_emergency_stop(void *pContext)
{
    app_fake_t *ptFake = (app_fake_t *)pContext;

    if (ptFake != NULL) {
        ptFake->wEmergencyStops++;
        ptFake->bPwmEnabled = false;
    }
}

static const foc_pwm_ops_t s_tPwmOps = {
    .fnDutyCommit = app_duty_commit,
    .fnPwmEnable = app_pwm_enable,
    .fnEmergencyStop = app_emergency_stop,
};

const foc_pwm_ops_t g_tFocPwmOps = {
    .fnDutyCommit = app_duty_commit,
    .fnPwmEnable = app_pwm_enable,
    .fnEmergencyStop = app_emergency_stop,
};

const foc_adc_ops_t g_tFocAdcOps = {
    .fnCalibrationBegin = app_adc_begin,
    .fnCalibrationStep = app_adc_step,
    .fnCurrentSample = app_adc_sample,
};

static foc_pid_params_t app_pi_params(float fLimit)
{
    foc_pid_params_t tParams = {
        .tKp = {0, FOC_SCALAR(0.20f)},
        .tKiTs = {0, FOC_SCALAR(0.005f)},
        .qOutputMinimum = FOC_SCALAR(-fLimit),
        .qOutputMaximum = FOC_SCALAR(fLimit),
        .qIntegratorMinimum = FOC_SCALAR(-fLimit),
        .qIntegratorMaximum = FOC_SCALAR(fLimit),
    };

    return tParams;
}

static foc_app_cfg_t app_config(void)
{
    foc_app_cfg_t tConfig = {0};

    tConfig.tMotorConfig.tMotorParams.chPolePairs = 7U;
    tConfig.tMotorConfig.tEncoderParams.qSpeedFilterAlpha =
        FOC_SCALAR(0.25f);
    tConfig.tMotorConfig.tEncoderParams.hwInvalidTimeout = 100U;
    tConfig.tMotorConfig.tControlCfg.tCurrentPiParams =
        app_pi_params(0.55f);
    tConfig.tMotorConfig.tControlCfg.tSpeedPiParams =
        app_pi_params(0.10f);
    tConfig.tMotorConfig.tControlCfg.qHighFrequencyPeriod =
        FOC_SCALAR(0.00005f);
    tConfig.tMotorConfig.tControlCfg.qPositionCalibrationCurrent =
        FOC_SCALAR(0.10f);
    tConfig.tMotorConfig.tControlCfg.qSpeedIqLimit = FOC_SCALAR(0.10f);
    tConfig.tMotorConfig.tControlCfg.hwCalibrationTimeoutTicks = 100U;
    tConfig.tMotorConfig.tControlCfg.wPositionCalibrationTicks = 5U;
    tConfig.tMotorConfig.ptAdcOps = &s_tAdcOps;
    tConfig.tMotorConfig.pAdcContext = &s_tFake;
    tConfig.tMotorConfig.ptPwmOps = &s_tPwmOps;
    tConfig.tMotorConfig.pPwmContext = &s_tFake;
    tConfig.tMotorConfig.tPosition.ptOps = &s_tPositionOps;
    tConfig.tMotorConfig.tPosition.pContext = &s_tFake;
    return tConfig;
}

static int test_app_only_owns_motor_domain(void)
{
    foc_app_cfg_t tConfig = app_config();
    motor_status_t tStatus = {0};

    memset(&s_tFake, 0, sizeof(s_tFake));
    s_tFake.bFeedbackValid = true;
    if (foc_app_Init((uintptr_t)&s_tApp,
                     (uintptr_t)&tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_HighFrequencyStep(&s_tApp.tMotor);
    motor_HighFrequencyStep(&s_tApp.tMotor);
    if (motor_SetCurrentReference(&s_tApp.tMotor, FOC_ZERO,
                                  FOC_SCALAR(0.05f)) != FOC_RESULT_OK ||
        motor_Start(&s_tApp.tMotor, FOC_MODE_CURRENT) != FOC_RESULT_OK) {
        return 1;
    }
    foc_app_HighFrequencyStep(&s_tApp);
    return motor_GetStatus(&s_tApp.tMotor, &tStatus) == FOC_RESULT_OK &&
                   tStatus.eLifecycle == MOTOR_STATE_RUNNING &&
                   tStatus.eMode == FOC_MODE_CURRENT &&
                   tStatus.bPwmEnabled
               ? 0 : 1;
}

static int test_app_foreground_is_explicit(void)
{
    foc_app_cfg_t tConfig = app_config();

    memset(&s_tFake, 0, sizeof(s_tFake));
    s_tFake.bFeedbackValid = true;
    if (foc_app_Init((uintptr_t)&s_tApp,
                     (uintptr_t)&tConfig) != FOC_RESULT_OK) {
        return 1;
    }
    motor_BackgroundStep(&s_tApp.tMotor);
    return s_tFake.wPolls == 1U ? 0 : 1;
}

int main(void)
{
    int nFailures = 0;
    int nResult = test_app_only_owns_motor_domain();

    printf("  app_motor_boundary: %s\n",
           nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    nResult = test_app_foreground_is_explicit();
    printf("  app_foreground: %s\n", nResult == 0 ? "PASS" : "FAIL");
    nFailures += nResult;
    printf("app framework: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
