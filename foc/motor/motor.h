/****************************************************************************
 * @file    motor.h
 * @brief   Single-motor FOC domain object contract.
 * @author  Codex
 * @date    2026-09-08
 ****************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_core.h"
#include "foc_port.h"
#include "foc_pid.h"
#include "motor_position.h"

typedef struct {
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    foc_scalar_t qHighFrequencyPeriod;
    uint16_t hwCalibrationTimeoutTicks;
    uint32_t wPositionCalibrationTicks;
} motor_control_cfg_t;

typedef struct {
    motor_params_t tMotorParams;
    motor_control_cfg_t tControlCfg;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;
} motor_cfg_t;

typedef enum {
    MOTOR_STATE_INITIALIZING = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_CALIBRATING,
    MOTOR_STATE_POSITION_CAL,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} motor_lifecycle_e;

typedef enum {
    MOTOR_COMMAND_NONE = 0,
    MOTOR_COMMAND_START,
    MOTOR_COMMAND_STOP,
    MOTOR_COMMAND_CLEAR_FAULT,
    MOTOR_COMMAND_ADC_CALIBRATION,
    MOTOR_COMMAND_POSITION_CALIBRATION,
} motor_command_e;

typedef enum {
    MOTOR_FAULT_NONE = 0U,
    MOTOR_FAULT_CALIBRATION_TIMEOUT = (1UL << 0),
    MOTOR_FAULT_CALIBRATION = (1UL << 1),
    MOTOR_FAULT_CURRENT_SAMPLE = (1UL << 2),
    MOTOR_FAULT_POSITION = (1UL << 3),
    MOTOR_FAULT_MATH = (1UL << 4),
    MOTOR_FAULT_DUTY_COMMIT = (1UL << 5),
    MOTOR_FAULT_PWM_ENABLE = (1UL << 6),
    MOTOR_FAULT_STATE = (1UL << 7),
    MOTOR_FAULT_POSITION_CAL = (1UL << 8),
} motor_fault_e;

typedef struct {
    motor_command_e ePending;
} motor_command_sync_t;

typedef struct {
    motor_params_t tParams;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;
    foc_scalar_t qHighFrequencyPeriod;
    uint16_t hwCalibrationTimeoutTicks;
    uint32_t wPositionCalibrationTicks;
    foc_core_state_t tCore;
    foc_pid_t tSpeedPi;
    foc_adc_calib_t tAdcCalibration;
    motor_command_sync_t tCommandSync;
    foc_core_command_t tCommand;
    foc_scalar_t qSpeedIqLimit;
    motor_lifecycle_e eLifecycle;
    motor_position_feedback_t tPositionFeedback;
    foc_scalar_t qIuLatest;     /**< 最近一拍 U 相电流（波形镜像缓存） */
    foc_scalar_t qIvLatest;     /**< 最近一拍 V 相电流（波形镜像缓存） */
    foc_scalar_t qIwLatest;     /**< 最近一拍 W 相电流（波形镜像缓存） */
    uint32_t wFaults;
    uint16_t hwCalibrationTicks;
    uint32_t wPositionCalibrationCount;
    bool bPwmEnabled;
    bool bStartAfterCalibration;
    bool bPositionCalAfterCalibration;
} motor_t;

typedef struct {
    motor_position_feedback_t tPosition;
    foc_dq_t tCurrent;
    foc_dq_t tVoltage;
    foc_duty_abc_t tDuty;
} motor_feedback_t;

typedef struct {
    motor_lifecycle_e eLifecycle;
    uint32_t wFaults;
    foc_core_command_t tCommand;
    bool bPwmEnabled;
} motor_status_t;

foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig);
void motor_HighFrequencyStep(motor_t *ptMotor);
void motor_ClockStep(motor_t *ptMotor);
void motor_BackgroundStep(motor_t *ptMotor);
foc_result_t motor_Start(motor_t *ptMotor,
                         const foc_core_command_t *ptCommand);
void motor_Stop(motor_t *ptMotor);
foc_result_t motor_ClearFault(motor_t *ptMotor);
foc_result_t motor_RequestAdcCalibration(motor_t *ptMotor);
foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);
foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);
foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qElectricalSpeed);
foc_result_t motor_GetFeedback(const motor_t *ptMotor,
                               motor_feedback_t *ptFeedback);
foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus);
foc_result_t motor_CaptureElectricalZero(motor_t *ptMotor);

/**
 * @brief Align the rotor with the fixed zero angle and capture the
 *        electrical zero through the active position provider.
 * @param ptMotor       Motor object.
 * @param qAlignCurrent D-axis align current reference (0 < q <= 1 pu).
 * @return FOC_RESULT_OK when the request was accepted. A provider without
 *         fnCaptureElectricalZero returns FOC_RESULT_DISABLED; an invalid
 *         state or argument returns FOC_RESULT_BUSY or INVALID_ARGUMENT.
 */
foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor,
                                              foc_scalar_t qAlignCurrent);

#endif /* MOTOR_H */
