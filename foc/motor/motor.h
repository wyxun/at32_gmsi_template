/****************************************************************************
 * @file    motor.h
 * @brief   Single-motor lifecycle, control state, and real-time object.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_core.h"
#include "foc_position.h"
#include "foc_port.h"

/**
 * @brief Physical motor metadata owned by Motor.
 * @note Rs/Ld/Lq are validated and retained as explicit motor metadata.
 */
typedef struct {
    uint8_t chPolePairs;
    uint32_t wResistanceMilliohm;
    uint32_t wInductanceDMicroHenry;
    uint32_t wInductanceQMicroHenry;
} motor_params_t;

typedef struct {
    foc_scalar_t qMaxPhaseCurrent;
    foc_scalar_t qMaxIq;
    foc_scalar_t qMaxSpeedReference;
    foc_scalar_t qMaxModulation;
} motor_limits_t;

typedef struct {
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    uint32_t wAdcCalibrationTimeoutSteps;
    uint32_t wAlignSteps;
    uint8_t chSpeedLoopDiv;
    foc_scalar_t qAlignCurrent;
} motor_control_cfg_t;

typedef foc_result_t (*motor_get_position_fn)(
    void *pContext, uint32_t wNowTick, foc_position_t *ptPosition);

typedef struct {
    motor_params_t tParams;
    motor_limits_t tLimits;
    motor_get_position_fn fnGetPosition;
    void *pPositionContext;
    motor_control_cfg_t tControl;
} motor_cfg_t;

typedef enum {
    MOTOR_STATE_INITIALIZING = 0,
    MOTOR_STATE_ADC_CAL,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT,
} motor_state_e;

typedef enum {
    MOTOR_FAULT_NONE = 0U,
    MOTOR_FAULT_ADC_CAL = 1U << 0,
    MOTOR_FAULT_ADC_SAMPLE = 1U << 1,
    MOTOR_FAULT_POSITION = 1U << 2,
    MOTOR_FAULT_MATH = 1U << 3,
    MOTOR_FAULT_PWM = 1U << 4,
    MOTOR_FAULT_ALIGN = 1U << 5,
} motor_fault_e;

typedef struct {
    motor_cfg_t tCfg;
    foc_core_state_t tCore;
    foc_pid_t tSpeedPi;
    foc_adc_calib_t tCalib;
    foc_core_command_t tCommand;
    foc_core_input_t tInput;
    foc_angle_t tElectricalZero;
    uint32_t wCalibrationSteps;
    uint32_t wAlignSteps;
    uint8_t chSpeedLoopCount;
    motor_state_e eState;
    uint32_t wFaults;
    bool bPwmEnabled;
} motor_t;

typedef struct {
    motor_state_e eState;
    uint32_t wFaults;
    foc_control_mode_e eMode;
    bool bPwmEnabled;
} motor_status_t;

/**
 * @brief Initialize a Motor object and start safe ADC calibration.
 * @param ptMotor Motor object.
 * @param ptConfig Motor configuration copied into the object.
 * @return FOC_RESULT_OK or an initialization error.
 */
foc_result_t motor_Init(motor_t *ptMotor, const motor_cfg_t *ptConfig);

/**
 * @brief Start a supported closed-loop control mode.
 * @param ptMotor Motor object.
 * @param eMode Requested voltage, current, or speed mode.
 * @return FOC_RESULT_OK or a state/safety error.
 */
foc_result_t motor_Start(motor_t *ptMotor, foc_control_mode_e eMode);

/**
 * @brief Stop the power stage and return to IDLE when safe.
 * @param ptMotor Motor object.
 * @return None.
 */
void motor_Stop(motor_t *ptMotor);

/**
 * @brief Clear a latched fault while PWM is already stopped.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a state error.
 */
foc_result_t motor_ClearFault(motor_t *ptMotor);

/**
 * @brief Set voltage references through the Motor API.
 * @param ptMotor Motor object.
 * @param qD D-axis voltage reference.
 * @param qQ Q-axis voltage reference.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetVoltageReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/**
 * @brief Set current references through the Motor API.
 * @param ptMotor Motor object.
 * @param qD D-axis current reference.
 * @param qQ Q-axis current reference.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetCurrentReference(motor_t *ptMotor,
                                       foc_scalar_t qD,
                                       foc_scalar_t qQ);

/**
 * @brief Set the speed reference through the Motor API.
 * @param ptMotor Motor object.
 * @param qSpeedReference Electrical speed reference.
 * @return FOC_RESULT_OK or an argument/state error.
 */
foc_result_t motor_SetSpeedReference(motor_t *ptMotor,
                                     foc_scalar_t qSpeedReference);

/**
 * @brief Request a non-blocking electrical-zero alignment sequence.
 * @param ptMotor Motor object.
 * @return FOC_RESULT_OK or a safety/state error.
 */
foc_result_t motor_RequestPositionCalibration(motor_t *ptMotor);

/**
 * @brief Execute one deterministic high-frequency Motor step.
 * @param ptMotor Motor object.
 * @param wNowTick Low 32 bits of the current system tick.
 * @return None.
 */
void motor_HighFrequencyStep(motor_t *ptMotor, uint32_t wNowTick);

/**
 * @brief Copy a safe status snapshot.
 * @param ptMotor Motor object.
 * @param ptStatus Output status.
 * @return FOC_RESULT_OK or FOC_RESULT_NULL.
 */
foc_result_t motor_GetStatus(const motor_t *ptMotor,
                             motor_status_t *ptStatus);

#endif /* MOTOR_H */
