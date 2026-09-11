/****************************************************************************
 * @file    foc_port_config.h
 * @brief   STM32G431 FOC hardware-source declarations.
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

#include <stdint.h>

#include "as5600.h"

extern as5600_t g_tFocAs5600;

int32_t foc_port_As5600Init(void *pContext);
int32_t foc_port_As5600Read(void *pContext, uint16_t *phwRawAngle);

#endif /* FOC_PORT_CONFIG_H */
