/****************************************************************************
 * @file    foc_port_config.h
 * @brief   STM32G431 static PositionPort binding for the FOC App.
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

#include "as5600.h"

extern as5600_sensor_t g_tFocPositionSensor;
extern const motor_position_ops_t g_tFocPositionOps;

#define FOC_PORT_DEFAULT_POSITION \
    { .ptOps = &g_tFocPositionOps, .pContext = &g_tFocPositionSensor }

#endif /* FOC_PORT_CONFIG_H */
