/**
 * @file  as5600.c
 * @brief Minimal AS5600 raw mechanical-angle hardware driver.
 */

#include "as5600.h"

#include <stddef.h>

int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic)
{
    if (ptThis == NULL || ptIic == NULL ||
        ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    ptThis->ptIic = ptIic;
    return 0;
}

int32_t as5600_ReadMechanicalAngle(as5600_t *ptThis,
                                   uint16_t *phwRawAngle)
{
    uint8_t chRegister = AS5600_REG_RAW_ANGLE_H;
    uint8_t achData[2] = {0U, 0U};
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || phwRawAngle == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL ||
        ptIic->fnWrite(ptIic->pPriv, &chRegister, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, achData, 2U) < 0) {
        return -1;
    }
    *phwRawAngle = (uint16_t)(((uint16_t)achData[0] << 8 | achData[1]) &
                              0x0FFFU);
    return 0;
}
