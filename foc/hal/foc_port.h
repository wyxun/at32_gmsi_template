/****************************************************************************
 * @file    foc_port.h
 * @brief   Direct ADC and PWM boundary for the FOC power stage.
 * @author  Codex
 * @date    2026-09-11
 ****************************************************************************/

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include "foc_types.h"

typedef enum {
    FOC_CALIBRATION_BUSY = 0,
    FOC_CALIBRATION_COMPLETE,
    FOC_CALIBRATION_FAILED,
} foc_calibration_state_e;

/**
 * @brief Begin ADC offset calibration.
 * @param ptCalibration Calibration state owned by Motor.
 * @return None.
 */
void foc_adc_CalibBegin(foc_adc_calib_t *ptCalibration);

/**
 * @brief Accumulate one ADC offset sample.
 * @param ptCalibration Calibration state owned by Motor.
 * @return Calibration progress or failure.
 */
foc_calibration_state_e foc_adc_CalibStep(
    foc_adc_calib_t *ptCalibration);

/**
 * @brief Sample and normalize all three phase currents.
 * @param ptCalibration Completed ADC calibration state.
 * @param ptCurrent Output three-phase current sample.
 * @return FOC_RESULT_OK or a safety/error result.
 */
foc_result_t foc_adc_Sample(const foc_adc_calib_t *ptCalibration,
                            foc_current_abc_t *ptCurrent);

/**
 * @brief Commit all three normalized PWM duties.
 * @param ptDuty Normalized U/V/W duties.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_pwm_SetDuty(const foc_duty_abc_t *ptDuty);

/**
 * @brief Enable the power stage after a valid duty has been committed.
 * @return FOC_RESULT_OK or a hardware error.
 */
foc_result_t foc_pwm_Enable(void);

/**
 * @brief Immediately disable the power stage.
 * @return None.
 */
void foc_pwm_Stop(void);

#endif /* FOC_PORT_H */
