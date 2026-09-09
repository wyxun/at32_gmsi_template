/****************************************************************************
 * @file    foc_types.h
 * @brief   Shared value types for the minimal FOC core
 ************************************************************************** */

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include "foc_angle.h"

typedef struct {
    foc_scalar_t qAlpha;    /**< α 轴分量 */
    foc_scalar_t qBeta;     /**< β 轴分量 */
} foc_ab_t;

typedef struct {
    foc_scalar_t qD;        /**< D 轴分量 */
    foc_scalar_t qQ;        /**< Q 轴分量 */
} foc_dq_t;

typedef struct {
    foc_scalar_t qU;        /**< U 相占空比 */
    foc_scalar_t qV;        /**< V 相占空比 */
    foc_scalar_t qW;        /**< W 相占空比 */
} foc_duty_abc_t;

typedef struct {
    uint32_t wOffsetU;      /**< U 相 ADC 偏移量 */
    uint32_t wOffsetV;      /**< V 相 ADC 偏移量 */
    uint32_t wOffsetW;      /**< W 相 ADC 偏移量 */
    uint64_t ullSumU;       /**< U 相校准累加值 */
    uint64_t ullSumV;       /**< V 相校准累加值 */
    uint64_t ullSumW;       /**< W 相校准累加值 */
    uint16_t hwSampleCount; /**< 当前采样点数计数 */
    bool     bIsCalibrated; /**< 是否已完成校准 */
} foc_adc_calib_t;

typedef enum {
    FOC_STATE_IDLE = 0,
    FOC_STATE_CALIBRATING,
    FOC_STATE_RUNNING,
    FOC_STATE_FAULT,
} foc_run_state_e;

typedef enum {
    FOC_COMMAND_NONE = 0,
    FOC_COMMAND_START,
    FOC_COMMAND_STOP,
    FOC_COMMAND_CLEAR_FAULT,
} foc_command_e;

typedef enum {
    FOC_MODE_VOLTAGE = 0,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_POSITION,
    FOC_MODE_MAX,
} foc_control_mode_e;

typedef struct {
    foc_scalar_t qIu;
    foc_scalar_t qIv;
    foc_scalar_t qIw;
    foc_angle_t  tElectricalAngle;
    foc_scalar_t qElectricalSpeed;
    bool         bAngleValid;
} foc_core_input_t;

typedef struct {
    foc_control_mode_e eMode;
    foc_dq_t           tVoltageReference;
    foc_dq_t           tCurrentReference;
    foc_scalar_t       qSpeedReference;
} foc_core_command_t;

#endif /* FOC_TYPES_H */
