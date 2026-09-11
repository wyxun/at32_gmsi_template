/****************************************************************************
 * @file    foc_port.c
 * @brief   STM32G431 FOC port: default ops-table implementations
 * @author  Codex
 * @date    2026-08-29
 *
 * 这是 MDI 挂载层：foc 内部只通过 ops 表访问硬件，具体 mdi/vendor
 * 挂钩只发生在本文件。换芯片 = 换本文件提供的 ops 表。
 ************************************************************************** */

#include "foc_port.h"

#include <stdint.h>

#include "userconfig.h"
#include "haladc.h"
#include "haltim1.h"
#include "mdi_hw.h"
#include "mdi/mdi.h"
#include "port_mdi.h"

#if defined(MDI_HW_HAS_I2C_ENCODER)
#include "as5600.h"
#endif

#define FOC_PORT_ADC_SAMPLES          512U
#define FOC_PORT_PWM_PERIOD           4250U
#define FOC_PORT_CURRENT_COUNTS_PU   1390U
#define FOC_PORT_OFFSET_MIN         20000U
#define FOC_PORT_OFFSET_MAX         60000U

/* ===== ADC：三相电流采样 + 零偏校准 ===== */

/** @brief Read the injected ADC results for all three phases. */
static void port_read_raw(uint32_t *pwRawU,
                          uint32_t *pwRawV,
                          uint32_t *pwRawW)
{
    *pwRawU = haladc_GetInjected(HALADC_ADC1, 0U);
    *pwRawV = haladc_GetInjected(HALADC_ADC2, 1U);
    *pwRawW = haladc_GetInjected(HALADC_ADC2, 0U);
}

/** @brief Convert one ADC offset delta into normalized current. */
static foc_scalar_t port_normalize_current(int32_t nDelta)
{
    int32_t nBase = (int32_t)FOC_PORT_CURRENT_COUNTS_PU;

    nDelta = nDelta > nBase ? nBase : nDelta;
    nDelta = nDelta < -nBase ? -nBase : nDelta;
#if defined(FOC_NUMERIC_FIXED)
    return (foc_scalar_t)(((int64_t)nDelta * FOC_Q_SCALE) / nBase);
#else
    return (foc_scalar_t)nDelta / (foc_scalar_t)nBase;
#endif
}

/** @brief Convert a normalized duty value to the TIM1 compare range. */
static uint32_t port_duty_to_counts(foc_scalar_t qDuty)
{
    qDuty = foc_sat(qDuty, FOC_ZERO, FOC_ONE);
#if defined(FOC_NUMERIC_FIXED)
    return (uint32_t)(((int64_t)qDuty * FOC_PORT_PWM_PERIOD) /
                      FOC_Q_SCALE);
#else
    return (uint32_t)(qDuty * (foc_scalar_t)FOC_PORT_PWM_PERIOD);
#endif
}

/** @brief Check the ADC offset range required by this power stage. */
static bool port_offsets_are_valid(const foc_adc_calib_t *ptCalibration)
{
    return ptCalibration->wOffsetU >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetU <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetV >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetV <= FOC_PORT_OFFSET_MAX &&
           ptCalibration->wOffsetW >= FOC_PORT_OFFSET_MIN &&
           ptCalibration->wOffsetW <= FOC_PORT_OFFSET_MAX;
}

/** @brief Finalize and validate the accumulated ADC offsets. */
static bool port_store_calibration(foc_adc_calib_t *ptCalibration)
{
    ptCalibration->wOffsetU = (uint32_t)(ptCalibration->ullSumU /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetV = (uint32_t)(ptCalibration->ullSumV /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->wOffsetW = (uint32_t)(ptCalibration->ullSumW /
                                         FOC_PORT_ADC_SAMPLES);
    ptCalibration->bIsCalibrated = port_offsets_are_valid(ptCalibration);
    return ptCalibration->bIsCalibrated;
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
    haltim1_StartAdcTrigger();
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
    return port_store_calibration(ptCalibration)
        ? FOC_CALIBRATION_COMPLETE : FOC_CALIBRATION_FAILED;
}

static foc_result_t port_current_sample(
    void *pContext,
    const foc_adc_calib_t *ptCalibration,
    foc_core_input_t *ptInput)
{
    uint32_t wRawU;
    uint32_t wRawV;
    uint32_t wRawW;

    (void)pContext;
    if (ptCalibration == NULL || ptInput == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!ptCalibration->bIsCalibrated) {
        return FOC_RESULT_SAFETY;
    }
    port_read_raw(&wRawU, &wRawV, &wRawW);
    ptInput->qIu = port_normalize_current(
        (int32_t)ptCalibration->wOffsetU - (int32_t)wRawU);
    ptInput->qIv = port_normalize_current(
        (int32_t)ptCalibration->wOffsetV - (int32_t)wRawV);
    ptInput->qIw = port_normalize_current(
        (int32_t)ptCalibration->wOffsetW - (int32_t)wRawW);
    return FOC_RESULT_OK;
}

/* ===== PWM：duty 提交 / 使能 / 急停 ===== */

static foc_result_t port_duty_commit(void *pContext,
                                     const foc_duty_abc_t *ptDuty)
{
    (void)pContext;
    if (ptDuty == NULL) {
        return FOC_RESULT_NULL;
    }
    return port_mdi_MotorPwmSetDuty3(
        port_duty_to_counts(ptDuty->qU),
        port_duty_to_counts(ptDuty->qV),
        port_duty_to_counts(ptDuty->qW)) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static foc_result_t port_pwm_enable(void *pContext, bool bEnable)
{
    (void)pContext;
    if (HW.ptMotorU == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return MDI_Enable(HW.ptMotorU, bEnable) == 0
        ? FOC_RESULT_OK : FOC_RESULT_INVALID_ARGUMENT;
}

static void port_emergency_stop(void *pContext)
{
    (void)pContext;
    if (HW.ptMotorU != NULL) {
        (void)MDI_Enable(HW.ptMotorU, false);
    }
}

/* ===== 默认 ops 表 ===== */

const foc_pwm_ops_t g_tFocPwmOps = {
    .fnDutyCommit    = port_duty_commit,
    .fnPwmEnable     = port_pwm_enable,
    .fnEmergencyStop = port_emergency_stop,
};

const foc_adc_ops_t g_tFocAdcOps = {
    .fnCalibrationBegin = port_calibration_begin,
    .fnCalibrationStep  = port_calibration_step,
    .fnCurrentSample    = port_current_sample,
};

#if defined(MDI_HW_HAS_I2C_ENCODER)
as5600_t g_tFocAs5600;

int32_t foc_port_As5600Init(void *pContext)
{
    as5600_t *ptAs5600 = (as5600_t *)pContext;

    if (ptAs5600 == NULL || HW.ptI2c1 == NULL) {
        return -1;
    }
    return as5600_Init(ptAs5600, HW.ptI2c1);
}

int32_t foc_port_As5600Read(void *pContext,
                            uint16_t *phwRawAngle)
{
    if (pContext == NULL || phwRawAngle == NULL) {
        return -1;
    }
    return as5600_ReadMechanicalAngle((as5600_t *)pContext, phwRawAngle);
}

#endif
