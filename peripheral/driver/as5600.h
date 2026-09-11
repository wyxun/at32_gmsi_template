/**
 * @file  as5600.h
 * @brief Minimal AS5600 raw mechanical-angle hardware driver.
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

#include "mdi/mdi.h"

#define AS5600_I2C_ADDR        0x36U
#define AS5600_REG_RAW_ANGLE_H 0x0CU

typedef struct {
    mdi_iic_t *ptIic;
} as5600_t;

/** @brief Initialize one AS5600 hardware instance. */
int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic);

/** @brief Read the raw 12-bit mechanical angle in the foreground. */
int32_t as5600_ReadMechanicalAngle(as5600_t *ptThis,
                                   uint16_t *phwRawAngle);

#endif /* AS5600_H */
