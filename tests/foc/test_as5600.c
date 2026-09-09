#include <stdio.h>
#include <string.h>

#include "as5600.h"

typedef struct {
    uint32_t wWriteCalls;
    uint32_t wReadCalls;
    uint8_t chStatus;
    uint16_t hwRawAngle;
} as5600_test_bus_t;

static int32_t test_iic_write(void *pPriv,
                              const uint8_t *pchData,
                              uint32_t wLen)
{
    as5600_test_bus_t *ptBus = (as5600_test_bus_t *)pPriv;

    if (ptBus == NULL || pchData == NULL || wLen != 1U) {
        return -1;
    }
    ptBus->wWriteCalls++;
    return 0;
}

static int32_t test_iic_read(void *pPriv,
                             uint8_t *pchBuf,
                             uint32_t wLen)
{
    as5600_test_bus_t *ptBus = (as5600_test_bus_t *)pPriv;

    if (ptBus == NULL || pchBuf == NULL) {
        return -1;
    }
    ptBus->wReadCalls++;
    if (wLen == 3U) {
        pchBuf[0] = ptBus->chStatus;
        pchBuf[1] = (uint8_t)(ptBus->hwRawAngle >> 8);
        pchBuf[2] = (uint8_t)ptBus->hwRawAngle;
        return 0;
    }
    memset(pchBuf, 0, wLen);
    return 0;
}

static int test_slow_update_and_fast_read_are_separate(void)
{
    as5600_test_bus_t tBus = {
        .chStatus = AS5600_STATUS_MD,
        .hwRawAngle = 512U,
    };
    mdi_iic_t tIic = {
        .pPriv = &tBus,
        .fnWrite = test_iic_write,
        .fnRead = test_iic_read,
        .fnIsBusy = NULL,
    };
    as5600_sensor_t tSensor = {0};
    foc_encoder_params_t tEncoderParams = {0};
    motor_params_t tMotorParams = {
        .chPolePairs = 4U,
    };
    motor_position_feedback_t tFeedback = {0};
    uint32_t wBusCalls = 0U;

    foc_encoder_DefaultParams(&tEncoderParams);
    if (as5600_sensor_Init(&tSensor, &tIic) != 0) {
        return 1;
    }
    if (g_tAs5600PositionOps.fnInit(
            &tSensor, &tMotorParams, FOC_SCALAR(0.00005f),
            &tEncoderParams) != FOC_RESULT_OK) {
        return 1;
    }
    if (g_tAs5600PositionOps.fnPoll(&tSensor) != FOC_RESULT_OK) {
        return 1;
    }
    wBusCalls = tBus.wWriteCalls + tBus.wReadCalls;
    if (wBusCalls == 0U ||
        g_tAs5600PositionOps.fnReadFeedback(&tSensor, &tFeedback) !=
            FOC_RESULT_OK || !tFeedback.bValid) {
        return 1;
    }
    if (tBus.wWriteCalls + tBus.wReadCalls != wBusCalls) {
        return 1;
    }
    return tFeedback.tElectricalAngle.wBam32 == 0x80000000U ? 0 : 1;
}

int main(void)
{
    int nFailures = test_slow_update_and_fast_read_are_separate();

    printf("as5600 provider: %s (%d failures)\n",
           nFailures == 0 ? "PASS" : "FAIL", nFailures);
    return nFailures;
}
