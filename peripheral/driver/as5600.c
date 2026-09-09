/**
 * @file  as5600.c
 * @brief AS5600 磁旋转编码器驱动（芯片无关，经 mdi_iic_t 访问总线）
 */

#include "as5600.h"

#include <string.h>

int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic)
{
    if (ptThis == NULL || ptIic == NULL ||
        ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    memset(ptThis, 0, sizeof(*ptThis));
    ptThis->ptIic = ptIic;
    return 0;
}

bool as5600_IsMagnetOk(uint8_t chStatus)
{
    return (chStatus & AS5600_STATUS_MD) != 0U &&
           (chStatus & (AS5600_STATUS_MH | AS5600_STATUS_ML)) == 0U;
}

int32_t as5600_ReadRawAngle(as5600_t *ptThis, uint16_t *phwAngle)
{
    uint8_t chReg = AS5600_REG_RAW_ANGLE_H;
    uint8_t achBuf[2] = {0U, 0U};
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || phwAngle == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, achBuf, 2U) < 0) {
        return -1;
    }
    *phwAngle = (uint16_t)(((uint16_t)achBuf[0] << 8 | achBuf[1]) & 0x0FFFU);
    return 0;
}

int32_t as5600_ReadStatus(as5600_t *ptThis, uint8_t *pchStatus)
{
    uint8_t chReg = AS5600_REG_STATUS;
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || pchStatus == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, pchStatus, 1U) < 0) {
        return -1;
    }
    return 0;
}

int32_t as5600_Update(as5600_t *ptThis)
{
    uint8_t chReg = AS5600_REG_STATUS;
    uint8_t achBuf[3] = {0U, 0U, 0U};
    mdi_iic_t *ptIic = NULL;

    if (ptThis == NULL || ptThis->ptIic == NULL) {
        return -1;
    }
    ptIic = ptThis->ptIic;
    if (ptIic->fnWrite == NULL || ptIic->fnRead == NULL) {
        return -1;
    }
    /* 一次传输连续读取 STATUS(0x0B) 与 RAW_ANGLE(0x0C, 0x0D) */
    if (ptIic->fnWrite(ptIic->pPriv, &chReg, 1U) < 0 ||
        ptIic->fnRead(ptIic->pPriv, achBuf, 3U) < 0) {
        ptThis->tSample.bValid = false;
        return -1;
    }
    ptThis->tSample.chStatus = achBuf[0];
    ptThis->tSample.hwRawAngle = (uint16_t)(((uint16_t)achBuf[1] << 8 |
                                             achBuf[2]) & 0x0FFFU);
    ptThis->tSample.bMagnetOk = as5600_IsMagnetOk(achBuf[0]);
    ptThis->tSample.bValid = true;
    ptThis->tSample.wSequence++;
    return 0;
}

void as5600_GetSample(const as5600_t *ptThis, as5600_sample_t *ptSample)
{
    uint32_t wSeqBefore = 0U;
    uint32_t wSeqAfter = 0U;

    if (ptThis == NULL || ptSample == NULL) {
        return;
    }
    /* 单写者（低频任务）/单读者（ISR）：序号双读校验，读到一致快照为止 */
    do {
        wSeqBefore = ptThis->tSample.wSequence;
        *ptSample  = ptThis->tSample;
        wSeqAfter  = ptThis->tSample.wSequence;
    } while (wSeqBefore != wSeqAfter);
}

static foc_result_t as5600_position_Init(
    void *pContext,
    const motor_params_t *ptMotor,
    foc_scalar_t qHighFrequencyPeriod,
    const foc_encoder_params_t *ptEncoderParams)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pContext;

    if (ptDev == NULL || ptMotor == NULL ||
        ptMotor->chPolePairs == 0U || ptEncoderParams == NULL) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (foc_encoder_Init(&ptDev->tObserver, ptEncoderParams,
                         ptMotor->chPolePairs,
                         qHighFrequencyPeriod) != FOC_RESULT_OK) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptDev->tMotorParams = *ptMotor;
    ptDev->tElectricalZero = (foc_angle_t){0U};
    return FOC_RESULT_OK;
}

static foc_result_t as5600_position_Poll(void *pContext)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pContext;

    return ptDev == NULL || as5600_Update(&ptDev->tDriver) != 0
               ? FOC_RESULT_SAFETY : FOC_RESULT_OK;
}

static void as5600_position_Reset(void *pContext)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pContext;

    if (ptDev != NULL) {
        foc_encoder_Reset(&ptDev->tObserver);
    }
}

static foc_result_t as5600_position_ReadFeedback(
    void *pContext,
    motor_position_feedback_t *ptFeedback)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pContext;
    as5600_sample_t tSample = {0};
    foc_encoder_sample_t tEncSample = {0};
    foc_encoder_output_t tEncOutput = {0};
    uint64_t llElectrical = 0U;
    uint32_t wElectricalAngle = 0U;

    if (ptDev == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    as5600_GetSample(&ptDev->tDriver, &tSample);
    if (!tSample.bValid) {
        ptFeedback->bValid = false;
        return FOC_RESULT_OK;
    }
    tEncSample.hwRawAngle = tSample.hwRawAngle;
    tEncSample.wSequence = tSample.wSequence;
    tEncSample.bMagnetOk = tSample.bMagnetOk;
    if (foc_encoder_Step(&ptDev->tObserver, &tEncSample,
                         &tEncOutput) != FOC_RESULT_OK ||
        !ptDev->tObserver.bValid) {
        ptFeedback->bValid = false;
        return FOC_RESULT_OK;
    }
    ptFeedback->tMechanicalAngle = tEncOutput.tMechanicalAngle;
    ptFeedback->qMechanicalSpeed = tEncOutput.qMechanicalSpeed;
    llElectrical = (uint64_t)tEncOutput.tMechanicalAngle.wBam32 *
                   (uint64_t)ptDev->tMotorParams.chPolePairs;
    wElectricalAngle = (uint32_t)llElectrical;
    if (ptDev->bDirectionInverted) {
        wElectricalAngle = 0U - wElectricalAngle;
    }
    ptFeedback->tElectricalAngle = foc_angle_add(
        (foc_angle_t){wElectricalAngle}, ptDev->tElectricalZero);
    ptFeedback->qElectricalSpeed = foc_mul_wide(
        tEncOutput.qMechanicalSpeed,
        FOC_SCALAR((float)ptDev->tMotorParams.chPolePairs));
    ptFeedback->bValid = true;
    return FOC_RESULT_OK;
}

static foc_result_t as5600_position_CaptureZero(void *pContext)
{
    as5600_sensor_t *ptDev = (as5600_sensor_t *)pContext;
    motor_position_feedback_t tFeedback = {0};

    if (ptDev == NULL) {
        return FOC_RESULT_NULL;
    }
    if (as5600_position_ReadFeedback(ptDev, &tFeedback) !=
            FOC_RESULT_OK ||
        !tFeedback.bValid) {
        return FOC_RESULT_DISABLED;
    }
    ptDev->tElectricalZero.wBam32 =
        0U - tFeedback.tElectricalAngle.wBam32;
    return FOC_RESULT_OK;
}

const motor_position_ops_t g_tAs5600PositionOps = {
    .fnInit = as5600_position_Init,
    .fnReset = as5600_position_Reset,
    .fnPoll = as5600_position_Poll,
    .fnReadFeedback = as5600_position_ReadFeedback,
    .fnCaptureZero = as5600_position_CaptureZero,
};

int32_t as5600_sensor_Init(as5600_sensor_t *ptSensor,
                           mdi_iic_t *ptIic)
{
    if (ptSensor == NULL || ptIic == NULL) {
        return -1;
    }
    memset(ptSensor, 0, sizeof(*ptSensor));
    if (as5600_Init(&ptSensor->tDriver, ptIic) != 0) {
        return -1;
    }
    return 0;
}
