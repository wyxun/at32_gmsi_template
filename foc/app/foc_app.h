/****************************************************************************
 * @file    foc_app.h
 * @brief   MODUS Class interface for the single-motor FOC application
 * @author  Codex
 * @date    2026-08-31
 ****************************************************************************/

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "modus.h"
#include "foc_core.h"
#include "foc_encoder.h"
#include "foc_pid.h"
#include "foc_port.h"
#include "motor.h"

typedef enum {
    FOC_APP_ANGLE_OPEN_LOOP = 0,
    FOC_APP_ANGLE_ENCODER,
} foc_app_angle_source_e;

typedef struct {
    foc_angle_t tElectricalZero;
    foc_angle_t tOpenLoopAngle;
    foc_scalar_t qOpenLoopSpeed;
    foc_scalar_t qOpenLoopTargetSpeed;
    uint32_t wCurrentAlignTicks;
    foc_app_angle_source_e eAngleSource;
    uint8_t chPolePairs;
    bool bEncoderEnabled;
    bool bDirectionInverted;
    bool bCurrentStartupAlign;
    bool bEncoderCalibrated;
} foc_app_position_t;

typedef struct {
    uint32_t wIsrMaxCycles;
    uint32_t wIsrSamples;
    uint32_t wLastPollMs;
    uint16_t hwConsecutivePollFails;
    uint32_t wLastReportMs;
    uint8_t chRunPt;
#if defined(FOC_NUMERIC_FLOAT)
    float fElectricalAngleTurns;
    float fEncoderMechanicalTurns;
#endif
#if defined(FOC_NUMERIC_FLOAT) && defined(MWAVEFORM_ENABLE) && \
    MWAVEFORM_ENABLE
    uint8_t chEncoderWaveform;
#endif
} foc_app_diagnostics_t;

typedef struct {
    foc_pid_params_t tCurrentPiParams;
    foc_pid_params_t tSpeedPiParams;
    foc_encoder_params_t tEncoderParams;
    foc_angle_t tElectricalZero;
    const foc_pwm_ops_t *ptPwmOps;
    const foc_adc_ops_t *ptAdcOps;
    foc_sensor_t tSensor;           /**< 位置/速度传感器接口 */
    uint8_t chFeedbackSlot;
    uint8_t *pchRingBuffer;
    uint16_t hwRingSize;
    bool bDirectionInverted;
} foc_app_cfg_t;

typedef struct {
    modus_base_t *ptBase;
    foc_sensor_t tSensor;           /**< 位置/速度传感器接口 */
    foc_app_position_t tPosition;
    foc_app_diagnostics_t tDiagnostics;
    motor_t tMotor;
} foc_app_t;

typedef struct {
    foc_run_state_e eState;
    uint32_t wFaults;
    foc_angle_t tElectricalAngle;
    foc_angle_t tElectricalZero;
    foc_scalar_t qElectricalSpeed;
    foc_dq_t tCurrent;
    foc_dq_t tVoltage;
    foc_duty_abc_t tDuty;
    foc_adc_calib_t tCalibration;
    foc_control_mode_e eMode;
    foc_dq_t tVoltageReference;
    foc_dq_t tCurrentReference;
    foc_scalar_t qSpeedReference;
    bool bPwmEnabled;
    bool bEncoderCalibrated;
} foc_status_t;

/**
 * @brief  Initialize a MODUS FOC application object.
 * @param  wObjectAddr    Address of a foc_app_t object.
 * @param  wObjectCfgAddr Address of a foc_app_cfg_t configuration.
 * @return FOC_RESULT_OK or a MODUS/FOC error code.
 */
int foc_app_Init(uintptr_t wObjectAddr, uintptr_t wObjectCfgAddr);

/**
 * @brief  Run one object-bound 20 kHz FOC control step.
 * @param  ptThis  FOC application object.
 * @return None.
 */
void foc_app_HighFrequencyStep(foc_app_t *ptThis);

/**
 * @brief  ADC interrupt adapter for the generated single App object.
 * @return None.
 */
void foc_app_HighFrequencyISR(void);

/**
 * @brief  Submit a start command to an App object.
 * @param  ptThis     FOC application object.
 * @param  ptCommand  Control mode and reference values.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_Start(foc_app_t *ptThis,
                           const foc_core_command_t *ptCommand);

/**
 * @brief  Stop an App object, performing the hardware emergency stop first.
 * @param  ptThis  FOC application object.
 * @return None.
 */
void foc_app_Stop(foc_app_t *ptThis);

/**
 * @brief  Clear an App fault after the power stage is disabled.
 * @param  ptThis  FOC application object.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_ClearFault(foc_app_t *ptThis);

/**
 * @brief  Set voltage references for a voltage-mode App object.
 * @param  ptThis  FOC application object.
 * @param  qD      D-axis voltage reference in per-unit.
 * @param  qQ      Q-axis voltage reference in per-unit.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetVoltageReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ);

/**
 * @brief  Set current references for a current-mode App object.
 * @param  ptThis  FOC application object.
 * @param  qD      D-axis current reference in per-unit.
 * @param  qQ      Q-axis current reference in per-unit.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetCurrentReference(foc_app_t *ptThis,
                                         foc_scalar_t qD,
                                         foc_scalar_t qQ);

/**
 * @brief  Set the electrical speed reference for speed mode.
 * @param  ptThis  FOC application object.
 * @param  qElectricalTurnPerSecond Electrical speed in e-turn/s.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_SetSpeedReference(
    foc_app_t *ptThis, foc_scalar_t qElectricalTurnPerSecond);

/**
 * @brief  Configure the open-loop angle and electrical speed before start.
 * @param  ptThis              FOC application object.
 * @param  tInitialAngle      Initial electrical angle.
 * @param  qElectricalSpeed   Open-loop electrical speed in e-turn/s.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_ConfigureOpenLoop(
    foc_app_t *ptThis,
    foc_angle_t tInitialAngle,
    foc_scalar_t qElectricalSpeed);

/**
 * @brief  Submit an asynchronous encoder electrical-zero calibration.
 * @param  ptThis  FOC application object.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_RequestEncoderCalibration(foc_app_t *ptThis);

/**
 * @brief  Read the primary position feedback (mechanical angle/speed).
 * @param  ptThis           FOC application object.
 * @param  ptMechanicalAngle Output mechanical angle in turns.
 * @param  pqMechanicalSpeed Output mechanical speed in turn/s.
 * @param  pbValid           Output feedback validity.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_GetFeedback(const foc_app_t *ptThis,
                                 foc_angle_t *ptMechanicalAngle,
                                 foc_scalar_t *pqMechanicalSpeed,
                                 bool *pbValid);

/**
 * @brief  Copy a consistent status snapshot from an App object.
 * @param  ptThis    FOC application object.
 * @param  ptStatus  Output status snapshot.
 * @return FOC_RESULT_OK or an error code.
 */
foc_result_t foc_app_GetStatus(const foc_app_t *ptThis,
                               foc_status_t *ptStatus);

#if defined(FOC_APP_TEST)
/**
 * @brief  Mark encoder calibration complete in a host test object.
 * @param  ptThis  FOC application object.
 * @return None.
 */
void foc_app_TestMarkEncoderCalibrated(foc_app_t *ptThis);

/**
 * @brief  Run the object speed-loop test hook.
 * @param  ptThis  FOC application object.
 * @return None.
 */
void foc_app_TestRun1kHz(foc_app_t *ptThis);

/**
 * @brief  Read a test object's commanded Q-axis current.
 * @param  ptThis  FOC application object.
 * @return Current Q-axis reference in per-unit.
 */
foc_scalar_t foc_app_TestGetCurrentIqReference(
    const foc_app_t *ptThis);
#endif

#endif /* FOC_APP_H */
