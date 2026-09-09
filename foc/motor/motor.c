/****************************************************************************
 * @file    motor.c
 * @brief   Single-motor FOC lifecycle and 20 kHz orchestration.
 * @author  Codex
 * @date    2026-09-09
 ****************************************************************************/

#include <stddef.h>

#include "motor.h"
#include "perf_counter.h"

#define MOTOR_SPEED_DIVIDER 20U
#define MOTOR_POLL_INTERVAL_MS 1U
#define MOTOR_POLL_BACKOFF_MS 100U

static bool motor_mode_is_valid(foc_control_mode_e eMode)
{
    return eMode < FOC_MODE_MAX;
}

static bool motor_reference_is_valid(foc_scalar_t qValue)
{
    return qValue >= FOC_NEG_ONE && qValue <= FOC_ONE;
}

static bool motor_config_is_valid(const motor_cfg_t *ptConfig)
{
    const foc_adc_ops_t *ptAdc = NULL;
    const foc_pwm_ops_t *ptPwm = NULL;

    if (ptConfig == NULL) {
        return false;
    }
    ptAdc = ptConfig->ptAdcOps;
    ptPwm = ptConfig->ptPwmOps;
    if (ptAdc == NULL || ptPwm == NULL ||
        ptConfig->tPosition.ptOps == NULL ||
        ptAdc->fnCalibrationBegin == NULL ||
        ptAdc->fnCalibrationStep == NULL ||
        ptAdc->fnCurrentSample == NULL ||
        ptPwm->fnDutyCommit == NULL || ptPwm->fnPwmEnable == NULL ||
        ptPwm->fnEmergencyStop == NULL) {
        return false;
    }
    if (ptConfig->tMotorParams.chPolePairs == 0U ||
        ptConfig->tControlCfg.qHighFrequencyPeriod <= FOC_ZERO ||
        ptConfig->tControlCfg.hwCalibrationTimeoutTicks == 0U ||
        ptConfig->tControlCfg.wPositionCalibrationTicks == 0U ||
        !motor_reference_is_valid(
            ptConfig->tControlCfg.qPositionCalibrationCurrent) ||
        ptConfig->tControlCfg.qPositionCalibrationCurrent <= FOC_ZERO ||
        !motor_reference_is_valid(ptConfig->tControlCfg.qSpeedIqLimit) ||
        ptConfig->tControlCfg.qSpeedIqLimit <= FOC_ZERO) {
        return false;
    }
    if (ptConfig->tEncoderParams.qSpeedFilterAlpha < FOC_ZERO ||
        ptConfig->tEncoderParams.qSpeedFilterAlpha > FOC_ONE ||
        ptConfig->tEncoderParams.hwInvalidTimeout == 0U) {
        return false;
    }
    if (ptConfig->tPosition.ptOps->fnInit == NULL ||
        ptConfig->tPosition.ptOps->fnPoll == NULL ||
        ptConfig->tPosition.ptOps->fnReadFeedback == NULL) {
        return false;
    }
    return true;
}

static void motor_enter_fault(motor_t *ptMotor, uint32_t wFault)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->tConfig.ptPwmOps->fnEmergencyStop(
        ptMotor->tConfig.pPwmContext);
    ptMotor->tLifecycle.wFaults |= wFault;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_FAULT;
}

static void motor_begin_adc_calibration(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    ptMotor->tConfig.ptAdcOps->fnCalibrationBegin(
        ptMotor->tConfig.pAdcContext,
        &ptMotor->tControl.tAdcCalibration);
    ptMotor->tLifecycle.hwCalibrationTicks = 0U;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_ADC_CAL;
}

static foc_result_t motor_activate(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    foc_core_Reset(&ptMotor->tControl.tCore);
    eResult = ptMotor->tConfig.ptPwmOps->fnDutyCommit(
        ptMotor->tConfig.pPwmContext, &ptMotor->tControl.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return eResult;
    }
    eResult = ptMotor->tConfig.ptPwmOps->fnPwmEnable(
        ptMotor->tConfig.pPwmContext, true);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_PWM_ENABLE);
        return eResult;
    }
    ptMotor->tSchedule.chSpeedDivider = 0U;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_RUNNING;
    return FOC_RESULT_OK;
}

static foc_result_t motor_activate_alignment(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    ptMotor->tControl.tCurrentReference.qD =
        ptMotor->tConfig.tControlCfg.qPositionCalibrationCurrent;
    ptMotor->tControl.tCurrentReference.qQ = FOC_ZERO;
    ptMotor->tControl.eControlMode = FOC_MODE_CURRENT;
    foc_core_Reset(&ptMotor->tControl.tCore);
    eResult = ptMotor->tConfig.ptPwmOps->fnDutyCommit(
        ptMotor->tConfig.pPwmContext,
        &ptMotor->tControl.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return eResult;
    }
    eResult = ptMotor->tConfig.ptPwmOps->fnPwmEnable(
        ptMotor->tConfig.pPwmContext, true);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_PWM_ENABLE);
        return eResult;
    }
    ptMotor->tLifecycle.wPositionCalibrationCount = 0U;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_ALIGN;
    return FOC_RESULT_OK;
}

static void motor_finish_calibration(motor_t *ptMotor)
{
    foc_calibration_state_e eState = FOC_CALIBRATION_BUSY;

    if (ptMotor == NULL) {
        return;
    }
    if (ptMotor->tLifecycle.hwCalibrationTicks < UINT16_MAX) {
        ptMotor->tLifecycle.hwCalibrationTicks++;
    }
    eState = ptMotor->tConfig.ptAdcOps->fnCalibrationStep(
        ptMotor->tConfig.pAdcContext,
        &ptMotor->tControl.tAdcCalibration);
    if (eState == FOC_CALIBRATION_BUSY) {
        if (ptMotor->tLifecycle.hwCalibrationTicks >=
            ptMotor->tConfig.tControlCfg.hwCalibrationTimeoutTicks) {
            motor_enter_fault(ptMotor,
                              MOTOR_FAULT_CALIBRATION_TIMEOUT);
        }
        return;
    }
    if (eState != FOC_CALIBRATION_COMPLETE) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_CALIBRATION);
        return;
    }
    ptMotor->tControl.tAdcCalibration.bIsCalibrated = true;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_IDLE;
}

static bool motor_read_position(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return false;
    }
    eResult = ptMotor->tConfig.tPosition.ptOps->fnReadFeedback(
        ptMotor->tConfig.tPosition.pContext,
        &ptMotor->tControl.tPositionFeedback);
    if (eResult != FOC_RESULT_OK ||
        !ptMotor->tControl.tPositionFeedback.bValid) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_POSITION);
        return false;
    }
    return true;
}

static void motor_update_speed_reference(motor_t *ptMotor)
{
    foc_scalar_t qIqReference = FOC_ZERO;

    if (ptMotor->tControl.eControlMode != FOC_MODE_SPEED) {
        return;
    }
    if (ptMotor->tSchedule.chSpeedDivider == 0U) {
        qIqReference = foc_pid_Step(
            &ptMotor->tControl.tSpeedPi,
            ptMotor->tControl.qSpeedReference,
            ptMotor->tControl.tPositionFeedback.qElectricalSpeed);
        ptMotor->tControl.tCurrentReference.qQ = foc_sat(
            qIqReference, -ptMotor->tConfig.tControlCfg.qSpeedIqLimit,
            ptMotor->tConfig.tControlCfg.qSpeedIqLimit);
    }
    ptMotor->tSchedule.chSpeedDivider++;
    if (ptMotor->tSchedule.chSpeedDivider >= MOTOR_SPEED_DIVIDER) {
        ptMotor->tSchedule.chSpeedDivider = 0U;
    }
}

static foc_result_t motor_control_step(motor_t *ptMotor)
{
    foc_core_command_t tCommand = {0};

    tCommand.eMode = ptMotor->tControl.eControlMode;
    tCommand.tVoltageReference = ptMotor->tControl.tVoltageReference;
    tCommand.tCurrentReference = ptMotor->tControl.tCurrentReference;
    tCommand.qSpeedReference = ptMotor->tControl.qSpeedReference;
    return foc_core_step(&ptMotor->tControl.tCore, &tCommand,
                         &ptMotor->tControl.tCycleInput);
}

static void motor_running_step(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = ptMotor->tConfig.ptAdcOps->fnCurrentSample(
        ptMotor->tConfig.pAdcContext,
        &ptMotor->tControl.tAdcCalibration,
        &ptMotor->tControl.tCycleInput);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
        return;
    }
    if (!motor_read_position(ptMotor)) {
        return;
    }
    motor_update_speed_reference(ptMotor);
    eResult = motor_control_step(ptMotor);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = ptMotor->tConfig.ptPwmOps->fnDutyCommit(
        ptMotor->tConfig.pPwmContext,
        &ptMotor->tControl.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
    }
}

static void motor_alignment_step(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_OK;

    eResult = ptMotor->tConfig.ptAdcOps->fnCurrentSample(
        ptMotor->tConfig.pAdcContext,
        &ptMotor->tControl.tAdcCalibration,
        &ptMotor->tControl.tCycleInput);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_CURRENT_SAMPLE);
        return;
    }
    ptMotor->tControl.tCycleInput.tElectricalAngle = (foc_angle_t){0U};
    ptMotor->tControl.tCycleInput.qElectricalSpeed = FOC_ZERO;
    ptMotor->tControl.tCycleInput.bAngleValid = true;
    eResult = motor_control_step(ptMotor);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_MATH);
        return;
    }
    eResult = ptMotor->tConfig.ptPwmOps->fnDutyCommit(
        ptMotor->tConfig.pPwmContext,
        &ptMotor->tControl.tCore.tDuty);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_DUTY_COMMIT);
        return;
    }
    if (ptMotor->tLifecycle.wPositionCalibrationCount <
        ptMotor->tConfig.tControlCfg.wPositionCalibrationTicks) {
        ptMotor->tLifecycle.wPositionCalibrationCount++;
    }
}

static void motor_finish_alignment(motor_t *ptMotor)
{
    foc_result_t eResult = FOC_RESULT_DISABLED;

    if (ptMotor->tConfig.tPosition.ptOps == NULL ||
        ptMotor->tConfig.tPosition.ptOps->fnCaptureZero == NULL) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_POSITION_CAL);
        return;
    }
    eResult = ptMotor->tConfig.tPosition.ptOps->fnCaptureZero(
        ptMotor->tConfig.tPosition.pContext);
    if (eResult != FOC_RESULT_OK) {
        motor_enter_fault(ptMotor, MOTOR_FAULT_POSITION_CAL);
        return;
    }
    ptMotor->tConfig.ptPwmOps->fnEmergencyStop(
        ptMotor->tConfig.pPwmContext);
    ptMotor->tLifecycle.bPositionCalibrated = true;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_IDLE;
}

static void motor_process_requests(motor_t *ptMotor)
{
    motor_request_e eRequest = MOTOR_REQUEST_NONE;

    eRequest = ptMotor->tLifecycle.ePendingRequest;
    ptMotor->tLifecycle.ePendingRequest = MOTOR_REQUEST_NONE;
    switch (eRequest) {
    case MOTOR_REQUEST_START:
        if (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->tLifecycle.wFaults == 0U &&
            ptMotor->tControl.tAdcCalibration.bIsCalibrated) {
            (void)motor_activate(ptMotor);
        }
        break;
    case MOTOR_REQUEST_ADC_CAL:
        if (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->tLifecycle.wFaults == 0U) {
            motor_begin_adc_calibration(ptMotor);
        }
        break;
    case MOTOR_REQUEST_ALIGN:
        if (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_IDLE &&
            ptMotor->tLifecycle.wFaults == 0U &&
            ptMotor->tControl.tAdcCalibration.bIsCalibrated) {
            (void)motor_activate_alignment(ptMotor);
        }
        break;
    case MOTOR_REQUEST_NONE:
    default:
        break;
    }
}

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig)
{
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL || ptConfig == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_config_is_valid(ptConfig)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    *ptMotor = (motor_t){0};
    ptMotor->tConfig.tParams = ptConfig->tMotorParams;
    ptMotor->tConfig.tControlCfg = ptConfig->tControlCfg;
    ptMotor->tConfig.tEncoderParams = ptConfig->tEncoderParams;
    ptMotor->tConfig.ptAdcOps = ptConfig->ptAdcOps;
    ptMotor->tConfig.pAdcContext = ptConfig->pAdcContext;
    ptMotor->tConfig.ptPwmOps = ptConfig->ptPwmOps;
    ptMotor->tConfig.pPwmContext = ptConfig->pPwmContext;
    ptMotor->tConfig.tPosition = ptConfig->tPosition;
    ptMotor->tSchedule.lLastPollMs = -1;
    ptMotor->tControl.eControlMode = FOC_MODE_VOLTAGE;
    ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_INITIALIZING;
    eResult = foc_pid_Init(&ptMotor->tControl.tCore.tIdPi,
                           &ptConfig->tControlCfg.tCurrentPiParams);
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tControl.tCore.tIqPi,
                               &ptConfig->tControlCfg.tCurrentPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = foc_pid_Init(&ptMotor->tControl.tSpeedPi,
                               &ptConfig->tControlCfg.tSpeedPiParams);
    }
    if (eResult == FOC_RESULT_OK) {
        eResult = ptMotor->tConfig.tPosition.ptOps->fnInit(
            ptMotor->tConfig.tPosition.pContext,
            &ptMotor->tConfig.tParams,
            ptMotor->tConfig.tControlCfg.qHighFrequencyPeriod,
            &ptMotor->tConfig.tEncoderParams);
    }
    if (eResult != FOC_RESULT_OK) {
        ptMotor->tConfig.ptPwmOps->fnEmergencyStop(
            ptMotor->tConfig.pPwmContext);
        ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_FAULT;
        return eResult;
    }
    foc_core_Reset(&ptMotor->tControl.tCore);
    return FOC_RESULT_OK;
}

void motor_HighFrequencyStep(motor_t *ptMotor)
{
    if (ptMotor == NULL) {
        return;
    }
    motor_process_requests(ptMotor);
    switch (ptMotor->tLifecycle.eLifecycle) {
    case MOTOR_STATE_INITIALIZING:
        motor_begin_adc_calibration(ptMotor);
        break;
    case MOTOR_STATE_ADC_CAL:
        motor_finish_calibration(ptMotor);
        break;
    case MOTOR_STATE_ALIGN:
        motor_alignment_step(ptMotor);
        break;
    case MOTOR_STATE_RUNNING:
        motor_running_step(ptMotor);
        break;
    case MOTOR_STATE_IDLE:
    case MOTOR_STATE_FAULT:
        break;
    default:
        motor_enter_fault(ptMotor, MOTOR_FAULT_STATE);
        break;
    }
}

foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_mode_is_valid(eMode)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    if (eMode == FOC_MODE_POSITION) {
        return FOC_RESULT_DISABLED;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_INITIALIZING ||
        ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_ADC_CAL) {
        eResult = FOC_RESULT_BUSY;
    } else if (ptMotor->tLifecycle.wFaults != 0U ||
               ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_IDLE) {
        eResult = FOC_RESULT_BUSY;
    } else {
        ptMotor->tControl.eControlMode = eMode;
        foc_pid_Reset(&ptMotor->tControl.tSpeedPi);
        ptMotor->tSchedule.chSpeedDivider = 0U;
        ptMotor->tLifecycle.ePendingRequest = MOTOR_REQUEST_START;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

void motor_Stop(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL) {
        return;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptMotor->tConfig.ptPwmOps->fnEmergencyStop(
        ptMotor->tConfig.pPwmContext);
    ptMotor->tLifecycle.ePendingRequest = MOTOR_REQUEST_NONE;
    ptMotor->tLifecycle.eLifecycle = ptMotor->tLifecycle.wFaults == 0U
                              ? MOTOR_STATE_IDLE : MOTOR_STATE_FAULT;
    perfc_port_resume_global_interrupt(tIrqState);
}

void motor_BackgroundStep(motor_t *ptMotor)
{
    int64_t lNowMs = 0;
    uint32_t wIntervalMs = MOTOR_POLL_INTERVAL_MS;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL ||
        ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_FAULT) {
        return;
    }
    if (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_ALIGN &&
        ptMotor->tLifecycle.wPositionCalibrationCount >=
            ptMotor->tConfig.tControlCfg.wPositionCalibrationTicks) {
        motor_finish_alignment(ptMotor);
        return;
    }
    lNowMs = get_system_ms();
    if (ptMotor->tSchedule.hwConsecutivePollFails > 0U) {
        wIntervalMs = MOTOR_POLL_BACKOFF_MS;
    }
    if (ptMotor->tSchedule.lLastPollMs >= 0 &&
        (uint64_t)(lNowMs - ptMotor->tSchedule.lLastPollMs) <
            (uint64_t)wIntervalMs) {
        return;
    }
    ptMotor->tSchedule.lLastPollMs = lNowMs;
    eResult = ptMotor->tConfig.tPosition.ptOps->fnPoll(
        ptMotor->tConfig.tPosition.pContext);
    if (eResult != FOC_RESULT_OK) {
        if (ptMotor->tSchedule.hwConsecutivePollFails < UINT16_MAX) {
            ptMotor->tSchedule.hwConsecutivePollFails++;
        }
    } else {
        ptMotor->tSchedule.hwConsecutivePollFails = 0U;
    }
}

foc_result_t motor_ClearFault(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_FAULT) {
        eResult = FOC_RESULT_BUSY;
    } else {
        ptMotor->tLifecycle.wFaults = MOTOR_FAULT_NONE;
        ptMotor->tLifecycle.eLifecycle = MOTOR_STATE_IDLE;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_RequestAdcCalibration(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_IDLE ||
        ptMotor->tLifecycle.wFaults != 0U) {
        eResult = FOC_RESULT_BUSY;
    } else {
        ptMotor->tLifecycle.ePendingRequest = MOTOR_REQUEST_ADC_CAL;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (ptMotor->tConfig.tPosition.ptOps == NULL ||
        ptMotor->tConfig.tPosition.ptOps->fnCaptureZero == NULL) {
        return FOC_RESULT_DISABLED;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    if (ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_IDLE ||
        ptMotor->tLifecycle.wFaults != 0U ||
        !ptMotor->tControl.tAdcCalibration.bIsCalibrated) {
        eResult = FOC_RESULT_BUSY;
    } else {
        ptMotor->tLifecycle.ePendingRequest = MOTOR_REQUEST_ALIGN;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

static foc_result_t motor_set_mode_reference(
    motor_t *ptMotor, foc_control_mode_e eMode)
{
    if (eMode == FOC_MODE_POSITION) {
        return FOC_RESULT_DISABLED;
    }
    if (ptMotor->tLifecycle.wFaults != 0U ||
        (ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_RUNNING &&
         ptMotor->tControl.eControlMode != eMode) ||
        (ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_IDLE &&
         ptMotor->tLifecycle.eLifecycle != MOTOR_STATE_RUNNING)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    return FOC_RESULT_OK;
}

foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_reference_is_valid(qD) || !motor_reference_is_valid(qQ)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    eResult = motor_set_mode_reference(ptMotor, FOC_MODE_VOLTAGE);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->tControl.tVoltageReference.qD = qD;
        ptMotor->tControl.tVoltageReference.qQ = qQ;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    if (!motor_reference_is_valid(qD) || !motor_reference_is_valid(qQ)) {
        return FOC_RESULT_INVALID_ARGUMENT;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    eResult = motor_set_mode_reference(ptMotor, FOC_MODE_CURRENT);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->tControl.tCurrentReference.qD = qD;
        ptMotor->tControl.tCurrentReference.qQ = qQ;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qElectricalSpeed)
{
    perfc_global_interrupt_status_t tIrqState = 0U;
    foc_result_t eResult = FOC_RESULT_OK;

    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    eResult = motor_set_mode_reference(ptMotor, FOC_MODE_SPEED);
    if (eResult == FOC_RESULT_OK) {
        ptMotor->tControl.qSpeedReference = qElectricalSpeed;
    }
    perfc_port_resume_global_interrupt(tIrqState);
    return eResult;
}

foc_result_t motor_SetPositionReference(motor_t *ptMotor,
                                         foc_angle_t tElectricalPosition)
{
    (void)tElectricalPosition;
    if (ptMotor == NULL) {
        return FOC_RESULT_NULL;
    }
    return FOC_RESULT_DISABLED;
}

foc_result_t motor_GetFeedback(const motor_t *ptMotor,
                               motor_feedback_t *ptFeedback)
{
    perfc_global_interrupt_status_t tIrqState = 0U;

    if (ptMotor == NULL || ptFeedback == NULL) {
        return FOC_RESULT_NULL;
    }
    tIrqState = perfc_port_disable_global_interrupt();
    ptFeedback->tPosition = ptMotor->tControl.tPositionFeedback;
    ptFeedback->tCurrent = ptMotor->tControl.tCore.tCurrent;
    ptFeedback->tVoltage = ptMotor->tControl.tCore.tVoltage;
    ptFeedback->tDuty = ptMotor->tControl.tCore.tDuty;
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
    ptStatus->eLifecycle = ptMotor->tLifecycle.eLifecycle;
    ptStatus->wFaults = ptMotor->tLifecycle.wFaults;
    ptStatus->eMode = ptMotor->tControl.eControlMode;
    ptStatus->tVoltageReference = ptMotor->tControl.tVoltageReference;
    ptStatus->tCurrentReference = ptMotor->tControl.tCurrentReference;
    ptStatus->qSpeedReference = ptMotor->tControl.qSpeedReference;
    ptStatus->tPositionReference = ptMotor->tControl.tPositionReference;
    ptStatus->tCalibration = ptMotor->tControl.tAdcCalibration;
    ptStatus->bPwmEnabled =
        ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_ALIGN ||
        ptMotor->tLifecycle.eLifecycle == MOTOR_STATE_RUNNING;
    ptStatus->bPositionCalibrated =
        ptMotor->tLifecycle.bPositionCalibrated;
    perfc_port_resume_global_interrupt(tIrqState);
    return FOC_RESULT_OK;
}
