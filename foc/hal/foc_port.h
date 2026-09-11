/****************************************************************************
 * @file    foc_port.h
 * @brief   FOC hardware port: ops-table abstraction for the power stage
 * @author  Codex
 * @date    2026-08-29
 *
 * 这是 FOC 与硬件的唯一边界。foc 内部（app/算法）只通过 ops 表访问
 * 硬件，不直接接触 MDI/vendor。默认实现由各芯片的 foc_port.c 提供
 * （MDI 挂载在实现层），host 测试可注入 stub ops 完成独立验证。
 *
 * 设计：ops 注入（策略模式）+ -flto 内联。20 kHz 高频路径经过 ops
 * 函数指针间接层，-flto 下内联为直接调用，成本趋零；多芯片/多传感
 * 器适配只更换 ops 表，foc 内部零改动。
 *
 * 位置结果由 foc_position.h 统一表达；位置后端由板级配置绑定。
 **************************************************************************/

#ifndef FOC_PORT_H
#define FOC_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_types.h"

typedef enum {
    FOC_CALIBRATION_BUSY = 0,
    FOC_CALIBRATION_COMPLETE,
    FOC_CALIBRATION_FAILED,
} foc_calibration_state_e;

/* ===== PWM ops ===== */
typedef struct {
    /** @brief Write all three PWM compare registers. */
    foc_result_t (*fnDutyCommit)(void *pContext,
                                 const foc_duty_abc_t *ptDuty);
    /** @brief Enable or disable the power-stage PWM output. */
    foc_result_t (*fnPwmEnable)(void *pContext, bool bEnable);
    /** @brief Disable the power stage immediately from any context. */
    void         (*fnEmergencyStop)(void *pContext);
} foc_pwm_ops_t;

/* ===== ADC ops（三相电流采样 + 零偏校准） ===== */
typedef struct {
    /** @brief Start a new three-phase current-offset calibration. */
    void (*fnCalibrationBegin)(void *pContext,
                               foc_adc_calib_t *ptCalibration);
    /** @brief Accumulate one current-offset sample. */
    foc_calibration_state_e (*fnCalibrationStep)(
        void *pContext,
        foc_adc_calib_t *ptCalibration);
    /** @brief Read and normalize the three phase currents. */
    foc_result_t (*fnCurrentSample)(
        void *pContext,
        const foc_adc_calib_t *ptCalibration,
        foc_core_input_t *ptInput);
} foc_adc_ops_t;

/* ===== 默认 ADC/PWM 实例（由各芯片 foc_port.c 提供） ===== */
extern const foc_pwm_ops_t  g_tFocPwmOps;
extern const foc_adc_ops_t  g_tFocAdcOps;

#endif /* FOC_PORT_H */
