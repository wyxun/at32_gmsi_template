/****************************************************************************
 * @file    foc_port_config.h
 * @brief   STM32G431 FOC hardware-source declarations.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

#define FOC_PORT_HAS_POSITION 1

#include <stdint.h>

void *foc_port_PositionContext(void);
int32_t foc_port_PositionInit(void *pContext);
int32_t foc_port_PositionRead(void *pContext, uint16_t *phwRawAngle);

#endif /* FOC_PORT_CONFIG_H */
