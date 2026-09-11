/****************************************************************************
 * @file    foc_app.h
 * @brief   MODUS composition and scheduling class for one FOC Motor.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "modus.h"
#include "foc_encoder.h"
#include "motor.h"

typedef struct {
    motor_cfg_t tMotorCfg;
    foc_encoder_cfg_t tEncoderCfg;
} foc_app_cfg_t;

typedef struct {
    modus_base_t *ptBase;
    motor_t tMotor;
    foc_encoder_t tEncoder;
    uint8_t chRunPt;
    int64_t lForegroundTimestamp;
    int64_t lBackoffTimestamp;
    bool bReady;
} foc_app_t;

/**
 * @brief Initialize the MODUS FOC composition object.
 * @param wObjectAddr Address of the generated foc_app_t object.
 * @param wObjectCfgAddr Address of the generated foc_app_cfg_t object.
 * @return MODUS_SUCCESS or a FOC error.
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/**
 * @brief Enter the Motor high-frequency path from the ADC interrupt.
 * @return None.
 */
void foc_app_HighFrequencyISR(void);

#endif /* FOC_APP_H */
