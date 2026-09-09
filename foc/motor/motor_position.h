/****************************************************************************
 * @file    motor_position.h
 * @brief   Cached position-feedback boundary owned by Motor.
 * @author  Codex
 * @date    2026-09-09
 ****************************************************************************/

#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

#include <stdbool.h>

#include "foc_encoder.h"
#include "foc_types.h"
#include "motor_params.h"

typedef struct {
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    foc_angle_t tMechanicalAngle;
    foc_scalar_t qMechanicalSpeed;
    bool bValid;
} motor_position_feedback_t;

typedef struct {
    foc_result_t (*fnInit)(void *pContext,
                           const motor_params_t *ptMotor,
                           foc_scalar_t qHighFrequencyPeriod,
                           const foc_encoder_params_t *ptEncoder);
    void (*fnReset)(void *pContext);
    foc_result_t (*fnPoll)(void *pContext);
    foc_result_t (*fnReadFeedback)(
        void *pContext, motor_position_feedback_t *ptFeedback);
    foc_result_t (*fnCaptureZero)(void *pContext);
} motor_position_ops_t;

typedef struct {
    const motor_position_ops_t *ptOps;
    void *pContext;
} motor_position_t;

#endif /* MOTOR_POSITION_H */
