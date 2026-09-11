/****************************************************************************
 * @file    foc_encoder.h
 * @brief   Mechanical angle and speed backend for an absolute sensor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_position.h"

typedef int32_t (*foc_encoder_sensor_init_fn)(void *pContext);
typedef int32_t (*foc_encoder_sensor_read_fn)(void *pContext,
                                              uint16_t *phwRawAngle);

typedef struct {
    foc_scalar_t qSpeedFilterAlpha;
    uint32_t wInvalidTimeoutUs;
    bool bDirectionInvert;
    foc_encoder_sensor_init_fn fnSensorInit;
    foc_encoder_sensor_read_fn fnSensorRead;
    void *pSensorContext;
} foc_encoder_cfg_t;

typedef struct {
    foc_position_t tPosition;
    uint32_t wSampleTick;
} foc_encoder_position_slot_t;

typedef struct {
    foc_encoder_cfg_t tCfg;
    foc_encoder_position_slot_t atPosition[2];
    uint32_t wTickFrequency;
    uint32_t wInvalidTimeoutTicks;
    uint32_t wLastSampleTick;
    uint16_t hwLastRawAngle;
    /* Publication metadata is atomic on the target and orders slot access. */
    volatile uint8_t chPublishedIndex;
    volatile bool bHasSample;
} foc_encoder_t;

/**
 * @brief Initialize the encoder and its bound raw sensor.
 * @param ptEncoder Encoder object.
 * @param ptConfig Sensor binding and mechanical filter configuration.
 * @return FOC_RESULT_OK, DISABLED, or an initialization error.
 */
foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_cfg_t *ptConfig);

/**
 * @brief Read one raw sensor sample and publish mechanical feedback.
 * @param ptEncoder Encoder object.
 * @return FOC_RESULT_OK or the sensor read error.
 */
foc_result_t foc_encoder_Update(foc_encoder_t *ptEncoder);

/**
 * @brief Read a consistent, age-checked mechanical position snapshot.
 * @param ptEncoder Encoder object.
 * @param wNowTick Low 32 bits of the current system tick.
 * @param ptPosition Output position snapshot.
 * @return FOC_RESULT_OK or a safety/argument error.
 */
foc_result_t foc_encoder_GetPosition(const foc_encoder_t *ptEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition);

/**
 * @brief Capture the current mechanical position for Motor alignment.
 * @param ptEncoder Encoder object.
 * @param wNowTick Low 32 bits of the current system tick.
 * @param ptPosition Output position snapshot.
 * @return FOC_RESULT_OK or a safety/argument error.
 */
foc_result_t foc_encoder_CaptureZero(const foc_encoder_t *ptEncoder,
                                     uint32_t wNowTick,
                                     foc_position_t *ptPosition);

#endif /* FOC_ENCODER_H */
