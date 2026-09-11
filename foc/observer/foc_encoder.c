/****************************************************************************
 * @file    foc_encoder.c
 * @brief   Mechanical angle and speed backend for an absolute sensor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#include "foc_encoder.h"

#include <limits.h>
#include <stddef.h>

#include "perf_counter.h"

#define FOC_ENCODER_RESOLUTION       4096U
#define FOC_ENCODER_HALF_RESOLUTION  2048
#define FOC_ENCODER_TIMEOUT_MIN_TICK 1U

/**
 * @brief Convert one 12-bit raw angle to BAM32.
 * @param hwRawAngle Raw 12-bit angle.
 * @return Mechanical BAM32 angle.
 */
static foc_angle_t encoder_raw_to_angle(uint16_t hwRawAngle)
{
    return (foc_angle_t){(uint32_t)hwRawAngle << 20};
}

/**
 * @brief Calculate the shortest signed raw-angle difference.
 * @param hwRawAngle Current raw angle.
 * @param hwLastRawAngle Previous raw angle.
 * @return Signed raw-angle delta.
 */
static int16_t encoder_raw_delta(uint16_t hwRawAngle,
                                 uint16_t hwLastRawAngle)
{
    int32_t nDelta = (int32_t)hwRawAngle - (int32_t)hwLastRawAngle;

    if (nDelta > FOC_ENCODER_HALF_RESOLUTION) {
        nDelta -= (int32_t)FOC_ENCODER_RESOLUTION;
    } else if (nDelta < -FOC_ENCODER_HALF_RESOLUTION) {
        nDelta += (int32_t)FOC_ENCODER_RESOLUTION;
    } else {
        /* The raw difference is already the shortest path. */
    }
    return (int16_t)nDelta;
}

/**
 * @brief Convert a raw delta and tick interval into turn/s.
 * @param nDelta Signed raw-angle delta.
 * @param wElapsedTicks Tick interval between sensor completions.
 * @param wTickFrequency System tick frequency.
 * @return Mechanical speed in the selected scalar backend.
 */
static foc_scalar_t encoder_sample_speed(int16_t nDelta,
                                         uint32_t wElapsedTicks,
                                         uint32_t wTickFrequency)
{
    if (wElapsedTicks == 0U || wTickFrequency == 0U) {
        return FOC_ZERO;
    }
#if defined(FOC_NUMERIC_FIXED)
    int64_t llSpeed = (int64_t)nDelta * FOC_Q_SCALE;

    llSpeed *= (int64_t)wTickFrequency;
    llSpeed /= (int64_t)FOC_ENCODER_RESOLUTION;
    llSpeed /= (int64_t)wElapsedTicks;
    if (llSpeed > INT32_MAX) {
        llSpeed = INT32_MAX;
    } else if (llSpeed < INT32_MIN) {
        llSpeed = INT32_MIN;
    } else {
        /* The result is representable by the fixed scalar. */
    }
    return (foc_scalar_t)llSpeed;
#else
    return ((foc_scalar_t)nDelta * (foc_scalar_t)wTickFrequency) /
           ((foc_scalar_t)FOC_ENCODER_RESOLUTION *
            (foc_scalar_t)wElapsedTicks);
#endif
}

/**
 * @brief Convert a tick age into a scalar turn displacement.
 * @param qSpeed Mechanical speed.
 * @param wAgeTicks Tick age of the published sample.
 * @param wTickFrequency System tick frequency.
 * @return Mechanical turn displacement.
 */
static foc_scalar_t encoder_age_turns(foc_scalar_t qSpeed,
                                      uint32_t wAgeTicks,
                                      uint32_t wTickFrequency)
{
#if defined(FOC_NUMERIC_FIXED)
    int64_t llTurns = (int64_t)qSpeed * (int64_t)wAgeTicks;

    if (wTickFrequency == 0U) {
        return FOC_ZERO;
    }
    llTurns /= (int64_t)wTickFrequency;
    if (llTurns > INT32_MAX) {
        llTurns = INT32_MAX;
    } else if (llTurns < INT32_MIN) {
        llTurns = INT32_MIN;
    } else {
        /* The extrapolated displacement is representable. */
    }
    return (foc_scalar_t)llTurns;
#else
    if (wTickFrequency == 0U) {
        return FOC_ZERO;
    }
    return qSpeed * (foc_scalar_t)wAgeTicks /
           (foc_scalar_t)wTickFrequency;
#endif
}

/**
 * @brief Apply board encoder direction to one mechanical sample.
 * @param ptEncoder Encoder configuration.
 * @param ptPosition Mechanical sample to transform.
 * @return None.
 */
static void encoder_apply_direction(const foc_encoder_t *ptEncoder,
                                    foc_position_t *ptPosition)
{
    if (ptEncoder->tCfg.bDirectionInvert) {
        ptPosition->tMechanicalAngle.wBam32 =
            0U - ptPosition->tMechanicalAngle.wBam32;
        ptPosition->qMechanicalSpeed = foc_sub_sat(
            FOC_ZERO, ptPosition->qMechanicalSpeed);
    }
}

/**
 * @brief Publish one fully constructed position slot.
 * @param ptEncoder Encoder object.
 * @param ptPosition Position value to publish.
 * @param wSampleTick Completion tick of the raw read.
 * @return None.
 * @note The inactive slot is complete before the volatile index is published.
 */
static void encoder_publish(foc_encoder_t *ptEncoder,
                            const foc_position_t *ptPosition,
                            uint32_t wSampleTick)
{
    uint8_t chNext = (uint8_t)(ptEncoder->chPublishedIndex ^ 1U);

    ptEncoder->atPosition[chNext].tPosition = *ptPosition;
    ptEncoder->atPosition[chNext].wSampleTick = wSampleTick;
    ptEncoder->chPublishedIndex = chNext;
}

foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_cfg_t *ptConfig)
{
    uint32_t wFrequency = 0U;
    uint64_t llTimeoutTicks = 0U;
    int32_t nResult = 0;

    if (ptEncoder == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    *ptEncoder = (foc_encoder_t){0};
    if (ptConfig->fnSensorInit == NULL ||
        ptConfig->fnSensorRead == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (ptConfig->qSpeedFilterAlpha < FOC_ZERO ||
        ptConfig->qSpeedFilterAlpha > FOC_ONE ||
        ptConfig->wInvalidTimeoutUs == 0U ||
        ptConfig->pSensorContext == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wFrequency = perfc_get_systimer_frequency();
    if (wFrequency == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    llTimeoutTicks = ((uint64_t)wFrequency *
                      (uint64_t)ptConfig->wInvalidTimeoutUs) /
                     1000000ULL;
    if (llTimeoutTicks < FOC_ENCODER_TIMEOUT_MIN_TICK) {
        llTimeoutTicks = FOC_ENCODER_TIMEOUT_MIN_TICK;
    }
    if (llTimeoutTicks > UINT32_MAX) {
        llTimeoutTicks = UINT32_MAX;
    }
    ptEncoder->tCfg = *ptConfig;
    ptEncoder->wTickFrequency = wFrequency;
    ptEncoder->wInvalidTimeoutTicks = (uint32_t)llTimeoutTicks;
    nResult = ptEncoder->tCfg.fnSensorInit(
        ptEncoder->tCfg.pSensorContext);
    if (nResult != 0) {
        *ptEncoder = (foc_encoder_t){0};
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

foc_result_t foc_encoder_Update(foc_encoder_t *ptEncoder)
{
    uint16_t hwRawAngle = 0U;
    uint32_t wSampleTick = 0U;
    uint32_t wElapsedTicks = 0U;
    foc_position_t tPosition = {0};
    foc_scalar_t qRawSpeed = FOC_ZERO;
    foc_scalar_t qPreviousSpeed = FOC_ZERO;
    foc_scalar_t qSpeedDelta = FOC_ZERO;
    int32_t nResult = 0;

    if (ptEncoder == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptEncoder->tCfg.fnSensorRead == NULL) {
        return FOC_RESULT_DISABLED;
    }
    nResult = ptEncoder->tCfg.fnSensorRead(
        ptEncoder->tCfg.pSensorContext, &hwRawAngle);
    if (nResult != 0) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    wSampleTick = (uint32_t)get_system_ticks();
    hwRawAngle = (uint16_t)(hwRawAngle & 0x0FFFU);
    tPosition.tMechanicalAngle = encoder_raw_to_angle(hwRawAngle);
    tPosition.qMechanicalSpeed = FOC_ZERO;
    tPosition.bValid = true;
    if (ptEncoder->bHasSample) {
        wElapsedTicks = wSampleTick - ptEncoder->wLastSampleTick;
        qRawSpeed = encoder_sample_speed(
            encoder_raw_delta(hwRawAngle, ptEncoder->hwLastRawAngle),
            wElapsedTicks, ptEncoder->wTickFrequency);
        qPreviousSpeed = ptEncoder->atPosition[
            ptEncoder->chPublishedIndex].tPosition.qMechanicalSpeed;
        if (ptEncoder->tCfg.bDirectionInvert) {
            qPreviousSpeed = foc_sub_sat(FOC_ZERO, qPreviousSpeed);
        }
        qSpeedDelta = foc_sub_sat(
            qRawSpeed, qPreviousSpeed);
        tPosition.qMechanicalSpeed = foc_add_sat(
            qPreviousSpeed,
            foc_mul_wide(qSpeedDelta, ptEncoder->tCfg.qSpeedFilterAlpha));
    }
    encoder_apply_direction(ptEncoder, &tPosition);
    encoder_publish(ptEncoder, &tPosition, wSampleTick);
    ptEncoder->hwLastRawAngle = hwRawAngle;
    ptEncoder->wLastSampleTick = wSampleTick;
    ptEncoder->bHasSample = true;
    return FOC_RESULT_OK;
}

foc_result_t foc_encoder_GetPosition(const foc_encoder_t *ptEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    uint8_t chPublished = 0U;
    uint32_t wAgeTicks = 0U;

    if (ptEncoder == NULL || ptPosition == NULL) {
        return FOC_RESULT_NULL;
    }
    chPublished = ptEncoder->chPublishedIndex;
    *ptPosition = ptEncoder->atPosition[chPublished].tPosition;
    if (!ptEncoder->bHasSample || !ptPosition->bValid) {
        return FOC_RESULT_SAFETY;
    }
    wAgeTicks = wNowTick -
                ptEncoder->atPosition[chPublished].wSampleTick;
    if (wAgeTicks > ptEncoder->wInvalidTimeoutTicks) {
        ptPosition->bValid = false;
        return FOC_RESULT_SAFETY;
    }
    ptPosition->tMechanicalAngle = foc_angle_add_scalar(
        ptPosition->tMechanicalAngle,
        encoder_age_turns(ptPosition->qMechanicalSpeed,
                          wAgeTicks, ptEncoder->wTickFrequency));
    return FOC_RESULT_OK;
}

foc_result_t foc_encoder_CaptureZero(const foc_encoder_t *ptEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition)
{
    return foc_encoder_GetPosition(ptEncoder, wNowTick, ptPosition);
}
