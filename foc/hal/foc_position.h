/****************************************************************************
 * @file    foc_position.h
 * @brief   Mechanical position value published by the encoder backend.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_POSITION_H
#define FOC_POSITION_H

#include <stdbool.h>

#include "foc_angle.h"

typedef struct {
    foc_angle_t tMechanicalAngle;
    foc_scalar_t qMechanicalSpeed;
    bool bValid;
} foc_position_t;

#endif /* FOC_POSITION_H */
