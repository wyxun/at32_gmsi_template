/****************************************************************************
 * @file    foc_encoder.c
 * @brief   Absolute magnetic-encoder (e.g. AS5600) angle/speed source
 *
 * 新样本到达：12 位码差分（int16 自然回卷）/ tick 数得原始速度，低通滤波；
 * 无新样本：按速度外插（低速不插）。输出机械角度/机械速度，电角度折算
 * 交给上层。磁铁失效或样本超时立即置无效。
 ****************************************************************************/

#include "foc_encoder.h"

#include <stddef.h>
#include <limits.h>
#include <string.h>

#if defined(FOC_NUMERIC_FLOAT)
#include <math.h>
#endif

#define FOC_ENCODER_RESOLUTION      4096U   /* 12-bit */
#define FOC_ENCODER_HALF_RESOLUTION 2048

/* 12 位原始码 → BAM32 机械角度（4096 → 2^32 精确映射，与数值后端无关） */
static foc_angle_t encoder_raw_to_angle(uint16_t hwRawAngle)
{
    foc_angle_t tAngle = { ((uint32_t)hwRawAngle << 20) };
    return tAngle;
}

/* 差分原始码（±2048 回卷） */
static int16_t encoder_raw_delta(uint16_t hwRawAngle,
                                 uint16_t hwLastRawAngle)
{
    int32_t nDelta = (int32_t)hwRawAngle - (int32_t)hwLastRawAngle;

    if (nDelta > FOC_ENCODER_HALF_RESOLUTION) {
        nDelta -= (int32_t)FOC_ENCODER_RESOLUTION;
    } else if (nDelta < -FOC_ENCODER_HALF_RESOLUTION) {
        nDelta += (int32_t)FOC_ENCODER_RESOLUTION;
    } else {
        /* The raw difference is already in the shortest direction. */
    }
    return (int16_t)nDelta;
}

static foc_scalar_t encoder_sample_speed(
    int16_t nDelta,
    uint16_t hwTicks,
    foc_scalar_t qHighFrequencyPeriod)
{
    foc_scalar_t qSpeed = FOC_ZERO;

    if (hwTicks == 0U || qHighFrequencyPeriod <= FOC_ZERO) {
        return FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FLOAT)
    qSpeed = (foc_scalar_t)nDelta /
             ((foc_scalar_t)FOC_ENCODER_RESOLUTION *
              (foc_scalar_t)hwTicks * qHighFrequencyPeriod);
#else
    foc_scalar_t qTurnPerTick = (foc_scalar_t)(
        ((int64_t)nDelta * FOC_Q_SCALE) /
        ((int32_t)FOC_ENCODER_RESOLUTION * (int32_t)hwTicks));
    if (foc_div_checked(qTurnPerTick, qHighFrequencyPeriod,
                        &qSpeed) != FOC_RESULT_OK) {
        qSpeed = FOC_ZERO;
    }
#endif
    return qSpeed;
}

/* 外推门限：|机械速度 × 极对数| ≥ 1 e-turn/s 才外推，避免低速量化噪声
   被放大成高频角度抖动（AS5600 低速 ±1.7 e-turn/s 量级） */
static bool encoder_should_extrapolate(foc_scalar_t qSpeed,
                                       uint8_t chPolePairs)
{
    if (chPolePairs == 0U) {
        return false;
    }
#if defined(FOC_NUMERIC_FIXED)
    return (int64_t)foc_abs(qSpeed) * (int64_t)chPolePairs >=
           (int64_t)FOC_Q_SCALE;
#else
    return fabsf(qSpeed) * (float)chPolePairs >= 1.0f;
#endif
}

static foc_angle_t encoder_extrapolated_angle(
    const foc_encoder_t *ptEncoder)
{
    foc_scalar_t qTurns = FOC_ZERO;

    if (encoder_should_extrapolate(ptEncoder->qMechanicalSpeed,
                                   ptEncoder->chPolePairs)) {
#if defined(FOC_NUMERIC_FIXED)
        int64_t llTurns = (int64_t)ptEncoder->qMechanicalSpeed *
                          (int64_t)ptEncoder->qHighFrequencyPeriod;
        llTurns = (llTurns / FOC_Q_SCALE) *
                  (int64_t)ptEncoder->hwTicksSinceSample;
        if (llTurns > INT32_MAX) {
            llTurns = INT32_MAX;
        } else if (llTurns < INT32_MIN) {
            llTurns = INT32_MIN;
        } else {
            /* The extrapolation remains representable in foc_scalar_t. */
        }
        qTurns = (foc_scalar_t)llTurns;
#else
        qTurns = ptEncoder->qMechanicalSpeed *
                 ptEncoder->qHighFrequencyPeriod *
                 (foc_scalar_t)ptEncoder->hwTicksSinceSample;
#endif
    }
    return foc_angle_add_scalar(ptEncoder->tMechanicalAngle, qTurns);
}

void foc_encoder_DefaultParams(foc_encoder_params_t *ptParams)
{
    if (ptParams != NULL) {
        ptParams->qSpeedFilterAlpha = FOC_SCALAR(0.25f);
        ptParams->hwInvalidTimeout = 100U;  /* 5 ms @20 kHz */
    }
}

foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_params_t *ptParams,
                              uint8_t chPolePairs,
                              foc_scalar_t qHighFrequencyPeriod)
{
    if (ptEncoder == NULL || ptParams == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptParams->qSpeedFilterAlpha < FOC_ZERO ||
        ptParams->qSpeedFilterAlpha > FOC_ONE ||
        ptParams->hwInvalidTimeout == 0U || chPolePairs == 0U ||
        qHighFrequencyPeriod <= FOC_ZERO) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    memset(ptEncoder, 0, sizeof(*ptEncoder));
    ptEncoder->tParams = *ptParams;
    ptEncoder->chPolePairs = chPolePairs;
    ptEncoder->qHighFrequencyPeriod = qHighFrequencyPeriod;
    return FOC_RESULT_OK;
}

void foc_encoder_Reset(foc_encoder_t *ptEncoder)
{
    foc_encoder_params_t tParams = {0};
    uint8_t chPolePairs = 0U;
    foc_scalar_t qHighFrequencyPeriod = FOC_ZERO;

    if (ptEncoder != NULL) {
        tParams = ptEncoder->tParams;
        chPolePairs = ptEncoder->chPolePairs;
        qHighFrequencyPeriod = ptEncoder->qHighFrequencyPeriod;
        memset(ptEncoder, 0, sizeof(*ptEncoder));
        ptEncoder->tParams = tParams;
        ptEncoder->chPolePairs = chPolePairs;
        ptEncoder->qHighFrequencyPeriod = qHighFrequencyPeriod;
    }
}

foc_result_t foc_encoder_Step(foc_encoder_t *ptEncoder,
                              const foc_encoder_sample_t *ptSample,
                              foc_encoder_output_t *ptOutput)
{
    uint16_t hwRawAngle;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptEncoder == NULL || ptSample == NULL || ptOutput == NULL) {
        return FOC_RESULT_NULL;
    }

    hwRawAngle = (uint16_t)(ptSample->hwRawAngle & 0x0FFFU);
    if (!ptSample->bMagnetOk) {
        ptEncoder->bValid = false;
        ptOutput->tMechanicalAngle = ptEncoder->tMechanicalAngle;
        ptOutput->qMechanicalSpeed = ptEncoder->qMechanicalSpeed;
        return FOC_RESULT_INVALID_ARGUMENT;
    }

    if (ptEncoder->hwTicksSinceSample < UINT16_MAX) {
        ptEncoder->hwTicksSinceSample++;
    }

    if (ptSample->wSequence != ptEncoder->wLastSequence) {
        /* 新样本 */
        if (!ptEncoder->bInitialized) {
            ptEncoder->bInitialized = true;
            ptEncoder->tMechanicalAngle =
                encoder_raw_to_angle(hwRawAngle);
            ptEncoder->qMechanicalSpeed = FOC_ZERO;
        } else if ((uint32_t)(ptSample->wSequence -
                              ptEncoder->wLastSequence) > 1U) {
            /* 采样间隙：停机/标定期间高频步未运行，本样本的增量跨越整个
               间隙而 hwTicksSinceSample 只计了当前步，速度必然虚高。
               只重设角度基准，不计算速度（避免启动瞬间速度尖峰）。 */
            ptEncoder->tMechanicalAngle =
                encoder_raw_to_angle(hwRawAngle);
            ptEncoder->qMechanicalSpeed = FOC_ZERO;
        } else {
            int16_t nDelta = encoder_raw_delta(hwRawAngle,
                                               ptEncoder->hwLastRawAngle);
            foc_scalar_t qRawSpeed =
                encoder_sample_speed(nDelta,
                                     ptEncoder->hwTicksSinceSample,
                                     ptEncoder->qHighFrequencyPeriod);
            foc_scalar_t qSpeedDelta =
                foc_sub_sat(qRawSpeed, ptEncoder->qMechanicalSpeed);

            ptEncoder->qMechanicalSpeed = foc_add_sat(
                ptEncoder->qMechanicalSpeed,
                foc_mul_wide(qSpeedDelta,
                             ptEncoder->tParams.qSpeedFilterAlpha));
            ptEncoder->tMechanicalAngle =
                encoder_raw_to_angle(hwRawAngle);
        }
        ptEncoder->hwLastRawAngle = hwRawAngle;
        ptEncoder->wLastSequence = ptSample->wSequence;
        ptEncoder->hwTicksSinceSample = 0U;
        ptEncoder->bValid = true;
    } else if (ptEncoder->hwTicksSinceSample >
               ptEncoder->tParams.hwInvalidTimeout) {
        /* 样本超时：缓存停止更新，输出不再可信 */
        ptEncoder->bValid = false;
        eResult = FOC_RESULT_INVALID_ARGUMENT;
    } else {
        /* 无新样本且未超时：角度按最后滤波速度外插（低速不插） */
    }

    ptOutput->tMechanicalAngle = ptEncoder->bValid
        ? encoder_extrapolated_angle(ptEncoder)
        : ptEncoder->tMechanicalAngle;
    ptOutput->qMechanicalSpeed = ptEncoder->qMechanicalSpeed;
    return eResult;
}
