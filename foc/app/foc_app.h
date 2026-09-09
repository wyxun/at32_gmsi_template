/****************************************************************************
 * @file    foc_app.h
 * @brief   MODUS and product glue for the single Motor object.
 * @author  Codex
 * @date    2026-09-09
 ****************************************************************************/

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdint.h>

#include "modus.h"
#include "foc_encoder.h"
#include "motor.h"

typedef struct {
    uint32_t wIsrMaxCycles;
    uint32_t wIsrSamples;
    uint32_t wLastReportMs;
    uint8_t chRunPt;
} foc_app_diagnostics_t;

typedef struct {
    motor_cfg_t tMotorConfig;
} foc_app_cfg_t;

typedef struct {
    modus_base_t *ptBase;
    foc_app_diagnostics_t tDiagnostics;
    motor_t tMotor;
} foc_app_t;

/** @brief Initialize one MODUS-bound App and its owned Motor. */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/** @brief Run one object-bound Motor high-frequency step. */
void foc_app_HighFrequencyStep(foc_app_t *ptThis);

/** @brief Adapter called by the board ADC interrupt. */
void foc_app_HighFrequencyISR(void);

#endif /* FOC_APP_H */
