/****************************************************************************
 * @file    motor.c
 * @brief   Single-motor FOC domain object and hard-real-time lifecycle.
 * @author  Codex
 * @date    2026-09-08
 ****************************************************************************/

#include <stddef.h>

#include "motor.h"
#include "perf_counter.h"

static foc_result_t motor_Activate(motor_t *ptMotor);

static bool motor_ConfigValid(const motor_cfg_t *ptConfig)
{
    if (ptConfig == NULL || ptConfig->ptAdcOps == NULL ||
        ptConfig->ptPwmOps == NULL || ptConfig->tPosition.ptOps == NULL ||
        ptConfig->ptAdcOps->fnCalibrationBegin == NULL ||
        ptConfig->ptAdcOps->fnCalibrationStep == NULL ||
        ptConfig->ptAdcOps->fnCurrentSample == NULL ||
        ptConfig->ptPwmOps->fnDutyCommit == NULL ||
        ptConfig->ptPwmOps->fnPwmEnable == NULL ||
        ptConfig->ptPwmOps->fnEmergencyStop == NULL ||
        ptConfig->tPosition.ptOps->fnInit == NULL ||
        ptConfig->tPosition.ptOps->fnRead == NULL ||
        ptConfig->tMotorParams.chPolePairs == 0U ||
        ptConfig->tControlCfg.qHighFrequencyPeriod <= FOC_ZERO ||
        ptConfig->tControlCfg.hwCalibrationTimeoutTicks == 0U) {
        return false;
    }
    return true;
}

static void motor_EnterFault(motor_t *ptMotor, uint32_t wFault)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
    ptMotor->bPwmEnabled = false;
    ptMotor->wFaults |= wFault;
    ptMotor->eLifecycle = MOTOR_STATE_FAULT;
}

static void motor_BeginCalibration(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->ptAdcOps->fnCalibrationBegin(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration);
    ptMotor->hwCalibrationTicks = 0U;
    ptMotor->eLifecycle = MOTOR_STATE_CALIBRATING;
}

/**
 * @brief Enable the power stage for the electrical-zero alignment run.
 * @param ptMotor Motor object.
 * @return None. Failures are latched through motor_EnterFault.
 * @note The align command must already be stored in tCommand.
 */
static void motor_ActivatePositionCalibration(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    foc_core_Reset(&ptMotor->tCore);
    if (ptMotor->ptPwmOps->fnDutyCommit(
            ptMotor->pPwmContext,
            &ptMotor->tCore.tDuty) != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return;
    }
    if (ptMotor->ptPwmOps->fnPwmEnable(
            ptMotor->pPwmContext, true) != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM_ENABLE);
        return;
    }
    ptMotor->bPwmEnabled = true;
    ptMotor->wPositionCalibrationCount = 0U;
    ptMotor->eLifecycle = MOTOR_STATE_POSITION_CAL;
}

static void motor_CalibrationStep(motor_t *ptMotor)
{
    foc_calibration_state_e eResult = FOC_CALIBRATION_BUSY;

    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->hwCalibrationTicks < UINT16_MAX) {
        ptMotor->hwCalibrationTicks++;
    }
    eResult = ptMotor->ptAdcOps->fnCalibrationStep(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration);
    if (eResult == FOC_CALIBRATION_BUSY) {
        if ((uint32_t)ptMotor->hwCalibrationTicks >=
            (uint32_t)ptMotor->hwCalibrationTimeoutTicks) {
            motor_EnterFault(ptMotor,
                             MOTOR_FAULT_CALIBRATION_TIMEOUT);
        }
        return;
    }
    if (eResult != FOC_CALIBRATION_COMPLETE) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_CALIBRATION);
        return;
    }
    ptMotor->tAdcCalibration.bIsCalibrated = true;
    ptMotor->eLifecycle = MOTOR_STATE_IDLE;
    if (ptMotor->bStartAfterCalibration) {
        ptMotor->bStartAfterCalibration = false;
        (void)motor_Activate(ptMotor);
    } else if (ptMotor->bPositionCalAfterCalibration) {
        /* 位置标定请求在 ADC 校准完成后进入对齐，而非普通运行。 */
        ptMotor->bPositionCalAfterCalibration = false;
        motor_ActivatePositionCalibration(ptMotor);
    }
}

static foc_result_t motor_Activate(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    foc_core_Reset(&ptMotor->tCore);
    eResult = ptMotor->ptPwmOps->fnDutyCommit(
        ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return eResult;
    }
    eResult = ptMotor->ptPwmOps->fnPwmEnable(
        ptMotor->pPwmContext, true);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_PWM_ENABLE);
        return eResult;
    }
    ptMotor->bPwmEnabled = true;
    ptMotor->eLifecycle = MOTOR_STATE_RUNNING;
    return FOC_RESULT_OK;
}

static bool motor_PositionStep(motor_t *ptMotor,
                               foc_core_input_t *ptInput)
{
    foc_result_t eResult = FOC_RESULT_OK;
    motor_position_feedback_t tFeedback = {0};

    if (ptMotor == NULL || ptInput == NULL) {
        return false;
    }
    if (ptMotor->tPosition.ptOps->fnObserve != NULL) {
        eResult = ptMotor->tPosition.ptOps->fnObserve(
            ptMotor->tPosition.pContext,
            &(foc_observer_input_t){0});
        if (eResult != FOC_RESULT_OK) {
            motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
            return false;
        }
    }
    eResult = ptMotor->tPosition.ptOps->fnRead(
        ptMotor->tPosition.pContext, &tFeedback);
    if (eResult != FOC_RESULT_OK || !tFeedback.bValid) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
        return false;
    }
    ptMotor->tPositionFeedback = tFeedback;
    ptInput->tElectricalAngle = tFeedback.tElectricalAngle;
    ptInput->qElectricalSpeed = tFeedback.qElectricalSpeed;
    ptInput->bAngleValid = true;
    return true;
}

static void motor_RunningStep(motor_t *ptMotor)
{
    foc_core_input_t tInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return;
    }
    eResult = ptMotor->ptAdcOps->fnCurrentSample(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration, &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
        return;
    }
    ptMotor->qIuLatest = tInput.qIu;
    ptMotor->qIvLatest = tInput.qIv;
    ptMotor->qIwLatest = tInput.qIw;
    if (!motor_PositionStep(ptMotor, &tInput)) {
        return;
    }
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = ptMotor->ptPwmOps->fnDutyCommit(
        ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
    }
}

/**
 * @brief Hold the rotor with the align current at the fixed zero angle.
 * @param ptMotor Motor object.
 * @return None. Failures are latched through motor_EnterFault.
 * @note The position provider is bypassed: alignment must not depend on
 *       the encoder feedback that is still being calibrated.
 */
static void motor_PositionCalibrationStep(motor_t *ptMotor)
{
    foc_core_input_t tInput = {0};
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return;
    }
    eResult = ptMotor->ptAdcOps->fnCurrentSample(
        ptMotor->pAdcContext, &ptMotor->tAdcCalibration, &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
        return;
    }
    ptMotor->qIuLatest = tInput.qIu;
    ptMotor->qIvLatest = tInput.qIv;
    ptMotor->qIwLatest = tInput.qIw;
    tInput.tElectricalAngle = (foc_angle_t){0U};
    tInput.qElectricalSpeed = FOC_ZERO;
    tInput.bAngleValid = true;
    eResult = foc_core_step(&ptMotor->tCore, &ptMotor->tCommand,
                            &tInput);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = ptMotor->ptPwmOps->fnDutyCommit(
        ptMotor->pPwmContext, &ptMotor->tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return;
    }
    if (ptMotor->wPositionCalibrationCount <
        ptMotor->wPositionCalibrationTicks) {
        ptMotor->wPositionCalibrationCount++;
    }
}

/**
 * @brief Capture the electrical zero while the rotor is held.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK after capture; other codes latch a position
 *         calibration fault. The power stage is always stopped here.
 */
static foc_result_t motor_CompletePositionCalibration(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_DISABLED;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tPosition.ptOps == NULL ||
        ptMotor->tPosition.ptOps->fnCaptureElectricalZero == NULL) {
        return FOC_RESULT_DISABLED;
    }
    eResult = ptMotor->tPosition.ptOps->fnCaptureElectricalZero(
        ptMotor->tPosition.pContext);
    if (eResult != FOC_RESULT_OK) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION_CAL);
        return eResult;
    }
    ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
    ptMotor->bPwmEnabled = false;
    ptMotor->eLifecycle = MOTOR_STATE_IDLE;
    return FOC_RESULT_OK;
}

static bool motor_ConsumeCommand(motor_t *ptMotor)
{
    motor_command_e eCommand = MOTOR_COMMAND_NONE;

    if (ptMotor == NULL) {
        return false;
    }
    eCommand = ptMotor->tCommandSync.ePending;
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_NONE;
    if (eCommand == MOTOR_COMMAND_START) {
        if (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->wFaults == 0U &&
            ptMotor->tAdcCalibration.bIsCalibrated) {
            (void)motor_Activate(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = true;
            motor_BeginCalibration(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = true;
        } else {
            /* Ignore requests outside a safe start state. */
        }
        return true;
    }
    if (eCommand == MOTOR_COMMAND_POSITION_CALIBRATION) {
        if (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->wFaults == 0U &&
            ptMotor->tAdcCalibration.bIsCalibrated &&
            ptMotor->wPositionCalibrationTicks != 0U) {
            motor_ActivatePositionCalibration(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bPositionCalAfterCalibration = true;
            motor_BeginCalibration(ptMotor);
        } else if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING &&
                   ptMotor->wFaults == 0U) {
            ptMotor->bPositionCalAfterCalibration = true;
        } else {
            /* Ignore requests outside a safe calibration state. */
        }
        return true;
    }
    if (eCommand == MOTOR_COMMAND_STOP) {
        motor_Stop(ptMotor);
        return true;
    }
    if (eCommand == MOTOR_COMMAND_ADC_CALIBRATION) {
        if (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->wFaults == 0U) {
            ptMotor->bStartAfterCalibration = false;
            motor_BeginCalibration(ptMotor);
        }
        return true;
    }
    return eCommand != MOTOR_COMMAND_NONE;
}

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_ConfigValid(ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptMotor = (motor_t){0};
    ptMotor->tParams = ptConfig->tMotorParams;
    ptMotor->ptAdcOps = ptConfig->ptAdcOps;
    ptMotor->pAdcContext = ptConfig->pAdcContext;
    ptMotor->ptPwmOps = ptConfig->ptPwmOps;
    ptMotor->pPwmContext = ptConfig->pPwmContext;
    ptMotor->tPosition = ptConfig->tPosition;
    ptMotor->qHighFrequencyPeriod =
        ptConfig->tControlCfg.qHighFrequencyPeriod;
    ptMotor->hwCalibrationTimeoutTicks =
        ptConfig->tControlCfg.hwCalibrationTimeoutTicks;
    ptMotor->wPositionCalibrationTicks =
        ptConfig->tControlCfg.wPositionCalibrationTicks;
    ptMotor->eLifecycle = MOTOR_STATE_INITIALIZING;
    eResult = foc_pid_Init(&ptMotor->tCore.tIdPi,
                           &ptConfig->tControlCfg.tCurrentPiParams);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tCore.tIqPi,
                               &ptConfig->tControlCfg.tCurrentPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tSpeedPi,
                               &ptConfig->tControlCfg.tSpeedPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = ptMotor->tPosition.ptOps->fnInit(
            ptMotor->tPosition.pContext, &ptMotor->tParams,
            ptMotor->qHighFrequencyPeriod);
    }
    if (eResult != FOC_RESULT_OK) {
        ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
        ptMotor->eLifecycle = MOTOR_STATE_FAULT;
        return eResult;
    }
    foc_core_Reset(&ptMotor->tCore);
    return FOC_RESULT_OK;
}

void motor_HighFrequencyStep(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    if (motor_ConsumeCommand(ptMotor)) {
        return;
    }
    switch (ptMotor->eLifecycle) {
    case MOTOR_STATE_INITIALIZING:
        motor_BeginCalibration(ptMotor);
        break;
    case MOTOR_STATE_CALIBRATING:
        motor_CalibrationStep(ptMotor);
        break;
    case MOTOR_STATE_POSITION_CAL:
        motor_PositionCalibrationStep(ptMotor);
        break;
    case MOTOR_STATE_RUNNING:
        motor_RunningStep(ptMotor);
        break;
    case MOTOR_STATE_IDLE:
    case MOTOR_STATE_FAULT:
        break;
    default:
        motor_EnterFault(ptMotor, MOTOR_FAULT_STATE);
        break;
    }
}

foc_result_t motor_Start(motor_t *ptMotor,
                         const foc_core_command_t *ptCommand)
{
    foc_scalar_t qRequestedIqLimit = FOC_ZERO;
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptCommand == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptCommand->eMode > FOC_MODE_SPEED) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    qRequestedIqLimit = foc_abs(ptCommand->tCurrentReference.qQ);
    if (ptCommand->eMode == FOC_MODE_SPEED &&
        qRequestedIqLimit > FOC_ONE) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if ((ptMotor->eLifecycle != MOTOR_STATE_IDLE &&
         ptMotor->eLifecycle != MOTOR_STATE_INITIALIZING &&
         ptMotor->eLifecycle != MOTOR_STATE_CALIBRATING) ||
        ptMotor->wFaults != 0U ||
        ptMotor->bPositionCalAfterCalibration) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    /* 与旧 App 每启动一次的速度环复位口径保持一致，避免积分器跨启动残留。 */
    foc_pid_Reset(&ptMotor->tSpeedPi);
    ptMotor->tCommand = *ptCommand;
    ptMotor->qSpeedIqLimit = qRequestedIqLimit > FOC_ZERO
        ? qRequestedIqLimit : FOC_ONE;
    if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING) {
        ptMotor->bStartAfterCalibration = true;
        motor_BeginCalibration(ptMotor);
    } else if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING) {
        ptMotor->bStartAfterCalibration = true;
    } else {
        ptMotor->tCommandSync.ePending = MOTOR_COMMAND_START;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

void motor_Stop(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptMotor->ptPwmOps->fnEmergencyStop(ptMotor->pPwmContext);
    ptMotor->bPwmEnabled = false;
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_NONE;
    /* 取消尚未完成的启动/标定意图，避免状态残留。 */
    ptMotor->bStartAfterCalibration = false;
    ptMotor->bPositionCalAfterCalibration = false;
    ptMotor->eLifecycle = ptMotor->wFaults == 0U
                              ? MOTOR_STATE_IDLE : MOTOR_STATE_FAULT;
    perfc_port_resume_global_interrupt(tIrqState);
}

void motor_ClockStep(motor_t *ptMotor)
{
    foc_scalar_t qIqReference = FOC_ZERO;
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptMotor->eLifecycle != MOTOR_STATE_RUNNING ||
        ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        return;
    }
    qIqReference = foc_pid_Step(&ptMotor->tSpeedPi,
                                ptMotor->tCommand.qSpeedReference,
                                ptMotor->tPositionFeedback.qElectricalSpeed);
    tIrqState = perfc_port_disable_global_interrupt();
    ptMotor->tCommand.tCurrentReference.qQ = foc_sat(
        qIqReference,
        -ptMotor->qSpeedIqLimit,
        ptMotor->qSpeedIqLimit);
    perfc_port_resume_global_interrupt(tIrqState);
}

void motor_BackgroundStep(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->eLifecycle == MOTOR_STATE_POSITION_CAL &&
        ptMotor->wPositionCalibrationCount >=
            ptMotor->wPositionCalibrationTicks) {
        (void)motor_CompletePositionCalibration(ptMotor);
    }
    if (ptMotor->eLifecycle == MOTOR_STATE_FAULT) {
        /* 保留已锁存故障码，慢速轮询不再覆盖。 */
        return;
    }
    if (ptMotor->tPosition.ptOps == NULL ||
        ptMotor->tPosition.ptOps->fnSlowUpdate == NULL) {
        return;
    }
    if (ptMotor->tPosition.ptOps->fnSlowUpdate(
            ptMotor->tPosition.pContext) != 0) {
        motor_EnterFault(ptMotor, MOTOR_FAULT_POSITION);
    }
}

foc_result_t motor_ClearFault(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eLifecycle != MOTOR_STATE_FAULT ||
        ptMotor->bPwmEnabled) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->wFaults = MOTOR_FAULT_NONE;
    ptMotor->eLifecycle = MOTOR_STATE_IDLE;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_RequestAdcCalibration(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->eLifecycle != MOTOR_STATE_IDLE ||
        ptMotor->wFaults != 0U) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_ADC_CALIBRATION;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor,
                                              foc_scalar_t qAlignCurrent)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tPosition.ptOps == NULL ||
        ptMotor->tPosition.ptOps->fnCaptureElectricalZero == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (qAlignCurrent <= FOC_ZERO || qAlignCurrent > FOC_ONE ||
        ptMotor->wPositionCalibrationTicks == 0U) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->wFaults != 0U ||
        ptMotor->eLifecycle == MOTOR_STATE_RUNNING ||
        ptMotor->eLifecycle == MOTOR_STATE_POSITION_CAL ||
        ptMotor->eLifecycle == MOTOR_STATE_FAULT ||
        ptMotor->bStartAfterCalibration ||
        ptMotor->tCommandSync.ePending != MOTOR_COMMAND_NONE) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_BUSY;
    }
    ptMotor->tCommand = (foc_core_command_t){0};
    ptMotor->tCommand.eMode = FOC_MODE_CURRENT;
    ptMotor->tCommand.tCurrentReference.qD = qAlignCurrent;
    ptMotor->tCommand.tCurrentReference.qQ = FOC_ZERO;
    if (ptMotor->eLifecycle == MOTOR_STATE_INITIALIZING ||
        (ptMotor->eLifecycle == MOTOR_STATE_IDLE &&
         !ptMotor->tAdcCalibration.bIsCalibrated)) {
        ptMotor->bPositionCalAfterCalibration = true;
        motor_BeginCalibration(ptMotor);
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_OK;
    }
    if (ptMotor->eLifecycle == MOTOR_STATE_CALIBRATING) {
        ptMotor->bPositionCalAfterCalibration = true;
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_OK;
    }
    ptMotor->tCommandSync.ePending = MOTOR_COMMAND_POSITION_CALIBRATION;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tCommand.eMode != FOC_MODE_VOLTAGE) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.tVoltageReference.qD = qD;
    ptMotor->tCommand.tVoltageReference.qQ = qQ;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tCommand.eMode != FOC_MODE_CURRENT) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.tCurrentReference.qD = qD;
    ptMotor->tCommand.tCurrentReference.qQ = qQ;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qElectricalSpeed)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tCommand.eMode != FOC_MODE_SPEED) {
        perfc_port_resume_global_interrupt(tIrqState);
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    ptMotor->tCommand.qSpeedReference = qElectricalSpeed;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetFeedback(const motor_t *ptMotor,
                               motor_feedback_t *ptFeedback)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptFeedback->tPosition = ptMotor->tPositionFeedback;
    ptFeedback->tCurrent = ptMotor->tCore.tCurrent;
    ptFeedback->tVoltage = ptMotor->tCore.tVoltage;
    ptFeedback->tDuty = ptMotor->tCore.tDuty;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptStatus == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptStatus->eLifecycle = ptMotor->eLifecycle;
    ptStatus->wFaults = ptMotor->wFaults;
    ptStatus->tCommand = ptMotor->tCommand;
    ptStatus->bPwmEnabled = ptMotor->bPwmEnabled;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}

foc_result_t motor_CaptureElectricalZero(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tPosition.ptOps == NULL ||
        ptMotor->tPosition.ptOps->fnCaptureElectricalZero == NULL) {
        return FOC_RESULT_DISABLED;
    }
    if (ptMotor->eLifecycle != MOTOR_STATE_RUNNING) {
        return FOC_RESULT_BUSY;
    }
    return ptMotor->tPosition.ptOps->fnCaptureElectricalZero(
        ptMotor->tPosition.pContext);
}
