/****************************************************************************
 * @file    foc_port.c
 * @brief   AT32F413 FOC port: ADC and PWM ops-table implementations.
 ****************************************************************************/

#include "foc_port.h"

#include <stddef.h>
#include <stdint.h>

#include "foc_config.h"
#include "haladc.h"
#include "halpwm.h"
#include "port_mdi.h"

#define FOC_PORT_ADC_SAMPLES 512U
#define FOC_PORT_CURRENT_BASE 2048U

static void port_read_raw(uint32_t *pwRawU,
                          uint32_t *pwRawV,
                          uint32_t *pwRawW)
{
    uint16_t hwRawU = 0U;
    uint16_t hwRawV = 0U;
    uint16_t hwRawW = 0U;

    haladc_GetPreemptRaw(&hwRawU, &hwRawV, &hwRawW);
    *pwRawU = (uint32_t)hwRawU;
    *pwRawV = (uint32_t)hwRawV;
    *pwRawW = (uint32_t)hwRawW;
}

static bool port_offsets_are_valid(const foc_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU > 0U &&
           ptCalibration->wOffsetU < 4096U &&
           ptCalibration->wOffsetV > 0U &&
           ptCalibration->wOffsetV < 4096U &&
           ptCalibration->wOffsetW > 0U &&
           ptCalibration->wOffsetW < 4096U;
}

static foc_scalar_t port_normalize_current(int32_t nDelta)
{
    int32_t nBase = (int32_t)FOC_PORT_CURRENT_BASE;

    nDelta = nDelta > nBase ? nBase : nDelta;
    nDelta = nDelta < -nBase ? -nBase : nDelta;
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta * FOC_Q_SCALE) / nBase);
#else
    return (foc_scalar_t)nDelta / (foc_scalar_t)nBase;
#endif
}

static uint32_t port_duty_to_counts(foc_scalar_t qDuty)
{
    qDuty = foc_sat(qDuty, FOC_ZERO, FOC_ONE);
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * PWM_PERIOD) / FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * (foc_scalar_t)PWM_PERIOD);
#endif
}

static void port_calibration_begin(void *pContext,
                                    foc_adc_calib_t *ptCalibration)
{
    (void)pContext;
    if (ptCalibration == NULL) {
        return;
    }
    ptCalibration->wOffsetU = 0U;
    ptCalibration->wOffsetV = 0U;
    ptCalibration->wOffsetW = 0U;
    ptCalibration->ullSumU = 0U;
    ptCalibration->ullSumV = 0U;
    ptCalibration->ullSumW = 0U;
    ptCalibration->hwSampleCount = 0U;
    ptCalibration->bIsCalibrated = false;
}

static foc_calibration_state_e port_calibration_step(
    void *pContext,
    foc_adc_calib_t *ptCalibration)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    (void)pContext;
    if (ptCalibration == NULL) {
        return FOC_CALIBRATION_FAILED;
    }
    if (ptCalibration->bIsCalibrated) {
        return FOC_CALIBRATION_COMPLETE;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptCalibration->ullSumU += wRawU;
    ptCalibration->ullSumV += wRawV;
    ptCalibration->ullSumW += wRawW;
    ptCalibration->hwSampleCount++;
    if (ptCalibration->hwSampleCount < FOC_PORT_ADC_SAMPLES) {
        return FOC_CALIBRATION_BUSY;
    }
    ptCalibration->wOffsetU = (uint32_t)(ptCalibration->ullSumU /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetV = (uint32_t)(ptCalibration->ullSumV /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetW = (uint32_t)(ptCalibration->ullSumW /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->bIsCalibrated = port_offsets_are_valid(ptCalibration);
    return ptCalibration->bIsCalibrated
        ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_FAILED;
}

static foc_result_t port_current_sample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    uint32_t wRawU = 0U;
    uint32_t wRawV = 0U;
    uint32_t wRawW = 0U;

    (void)pContext;
    if (ptCalibration == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptCalibration->bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptInput->qIu = port_normalize_current(
        (int32_t)wRawU - (int32_t)ptCalibration->wOffsetU);
    ptInput->qIv = port_normalize_current(
        (int32_t)wRawV - (int32_t)ptCalibration->wOffsetV);
    ptInput->qIw = port_normalize_current(
        (int32_t)wRawW - (int32_t)ptCalibration->wOffsetW);
    return FOC_RESULT_OK;
}

static foc_result_t port_duty_commit(void *pContext,
                                     const foc_duty_abc_t *ptDuty)
{
    (void)pContext;
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    halpwm_SetDuty(port_duty_to_counts(ptDuty->qU),
                   port_duty_to_counts(ptDuty->qV),
                   port_duty_to_counts(ptDuty->qW));
    return FOC_RESULT_OK;
}

static foc_result_t port_pwm_enable(void *pContext, bool bEnable)
{
    (void)pContext;
    if (bEnable) {
        halpwm_Start();
    } else {
        halpwm_Stop();
    }
    return FOC_RESULT_OK;
}

static void port_emergency_stop(void *pContext)
{
    (void)pContext;
    halpwm_Stop();
}

const foc_pwm_ops_t g_tFocPwmOps = {
    .fnDutyCommit = port_duty_commit,
    .fnPwmEnable = port_pwm_enable,
    .fnEmergencyStop = port_emergency_stop,
};

const foc_adc_ops_t g_tFocAdcOps = {
    .fnCalibrationBegin = port_calibration_begin,
    .fnCalibrationStep = port_calibration_step,
    .fnCurrentSample = port_current_sample,
};

/* This board has no registered position sensor. Runtime FOC is disabled. */
