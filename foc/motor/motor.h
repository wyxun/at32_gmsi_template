/****************************************************************************
 * @file    motor.h
 * @brief   Single-motor FOC domain object contract.
 * @author  Codex
 * @date    2026-09-09
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
    foc_scalar_t qPositionCalibrationCurrent;
    foc_scalar_t qSpeedIqLimit;
    uint16_t hwCalibrationTimeoutTicks;
    uint32_t wPositionCalibrationTicks;
} motor_control_cfg_t;

typedef struct {
    motor_params_t tMotorParams;
    motor_control_cfg_t tControlCfg;
    foc_encoder_params_t tEncoderParams;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;
} motor_cfg_t;

typedef enum {
    MOTOR_STATE_INITIALIZING = 0,
    MOTOR_STATE_ADC_CAL,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} motor_lifecycle_e;

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

typedef enum {
    MOTOR_REQUEST_NONE = 0,
    MOTOR_REQUEST_START,
    MOTOR_REQUEST_ADC_CAL,
    MOTOR_REQUEST_ALIGN,
} motor_request_e;

typedef struct {
    motor_params_t tParams;
    motor_control_cfg_t tControlCfg;
    foc_encoder_params_t tEncoderParams;
    const foc_adc_ops_t *ptAdcOps;
    void *pAdcContext;
    const foc_pwm_ops_t *ptPwmOps;
    void *pPwmContext;
    motor_position_t tPosition;
} motor_config_runtime_t;

typedef struct {
    foc_core_state_t tCore;
    foc_pid_t tSpeedPi;
    foc_adc_calib_t tAdcCalibration;
    foc_core_input_t tCycleInput;
    motor_position_feedback_t tPositionFeedback;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReference;
    foc_angle_t tPositionReference;
    foc_control_mode_e eControlMode;
} motor_control_runtime_t;

typedef struct {
    motor_lifecycle_e eLifecycle;
    uint32_t wFaults;
    uint16_t hwCalibrationTicks;
    uint32_t wPositionCalibrationCount;
    bool bPositionCalibrated;
    motor_request_e ePendingRequest;
} motor_lifecycle_runtime_t;

typedef struct {
    int64_t lLastPollMs;
    uint16_t hwConsecutivePollFails;
    uint8_t chSpeedDivider;
} motor_schedule_runtime_t;

typedef struct {
    motor_config_runtime_t tConfig;
    motor_control_runtime_t tControl;
    motor_lifecycle_runtime_t tLifecycle;
    motor_schedule_runtime_t tSchedule;
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
    foc_control_mode_e eMode;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReference;
    foc_angle_t tPositionReference;
    foc_adc_calib_t tCalibration;
    bool bPwmEnabled;
    bool bPositionCalibrated;
} motor_status_t;

/** @brief Initialize one Motor object and its registered providers. */
foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig);

/** @brief Run one ADC/PWM control tick in the 20 kHz context. */
void motor_HighFrequencyStep(motor_t *ptMotor);

/** @brief Run the foreground position poll and completion work. */
void motor_BackgroundStep(motor_t *ptMotor);

/** @brief Select a control mode and request a safe start. */
foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode);

/** @brief Immediately stop PWM and converge software state to safe idle. */
void motor_Stop(motor_t *ptMotor);

/** @brief Clear a latched fault while PWM is disabled. */
foc_result_t motor_ClearFault(motor_t *ptMotor);

/** @brief Request ADC offset calibration. */
foc_result_t motor_RequestAdcCalibration(motor_t *ptMotor);

/** @brief Request fixed-current position alignment and zero capture. */
foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor);

/** @brief Set the voltage reference for voltage mode. */
foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/** @brief Set the current reference for current mode. */
foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/** @brief Set the electrical speed reference for speed mode. */
foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qElectricalSpeed);

/** @brief Set the electrical position reference for position mode. */
foc_result_t motor_SetPositionReference(motor_t *ptMotor,
                                         foc_angle_t tElectricalPosition);

/** @brief Copy one coherent Motor feedback view. */
foc_result_t motor_GetFeedback(const motor_t *ptMotor,
                               motor_feedback_t *ptFeedback);

/** @brief Copy one coherent Motor status view. */
foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus);

#endif /* MOTOR_H */
