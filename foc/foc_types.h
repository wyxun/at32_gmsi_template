/****************************************************************************
 * @file    foc_types.h
 * @brief   Shared value types for the minimal FOC core.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include "foc_angle.h"

typedef struct {
    foc_scalar_t qAlpha;
    foc_scalar_t qBeta;
} foc_ab_t;

typedef struct {
    foc_scalar_t qD;
    foc_scalar_t qQ;
} foc_dq_t;

typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_duty_abc_t;

typedef struct {
    foc_scalar_t qU;
    foc_scalar_t qV;
    foc_scalar_t qW;
} foc_current_abc_t;

typedef struct {
    uint32_t wOffsetU;
    uint32_t wOffsetV;
    uint32_t wOffsetW;
    uint64_t ullSumU;
    uint64_t ullSumV;
    uint64_t ullSumW;
    uint16_t hwSampleCount;
    bool bIsCalibrated;
} foc_adc_calib_t;

typedef enum {
    FOC_MODE_VOLTAGE = 0,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION,
    FOC_MODE_MAX,
} foc_control_mode_e;

typedef struct {
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool bAngleValid;
} foc_core_input_t;

typedef struct {
    foc_control_mode_e eMode;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReference;
} foc_core_command_t;

#endif /* FOC_TYPES_H */
