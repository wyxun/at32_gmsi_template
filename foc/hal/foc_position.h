/****************************************************************************
 * @file    foc_position.h
 * @brief   Common position result returned by every FOC position backend.
 ****************************************************************************/

#ifndef FOC_POSITION_H
#define FOC_POSITION_H

#include "foc_angle.h"

typedef struct {
    foc_angle_t tMechanicalAngle;
    foc_scalar_t qMechanicalSpeed;
    foc_angle_t tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool bValid;
} foc_position_t;

#endif /* FOC_POSITION_H */
