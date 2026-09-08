/*******************************************************************************
 * @file    foc_config.h
 * @brief   Universal-FOC 框架统一配置入口
 *          所有编译期可调参数集中于此，其他模块通过 #include "foc_config.h" 读取
 ******************************************************************************/

#ifndef __FOC_CONFIG_H__
#define __FOC_CONFIG_H__

#if defined(FOC_NUMERIC_FLOAT) && defined(FOC_NUMERIC_FIXED)
#error "Select only one FOC numeric backend"
#elif !defined(FOC_NUMERIC_FLOAT) && !defined(FOC_NUMERIC_FIXED)
#error "Select FOC_NUMERIC_FLOAT or FOC_NUMERIC_FIXED"
#endif

/* Compatibility only. New code selects behavior through foc_numeric.h. */
#if defined(FOC_NUMERIC_FLOAT)
#define FOC_USE_FPU_HARDWARE        1       /**< 使用硬件 FPU */
#else
#define FOC_USE_FPU_HARDWARE        0       /**< 不使用硬件 FPU */
#endif

#define FOC_OFFSET_CALIB_TIMES      200U    /**< ADC 偏移校准采样次数 */

#define FOC_DEFAULT_SENSING_TOPOLOGY    SENSING_TOPOLOGY_3P  /**< 默认采样拓扑 */

#define FOC_HW_OCP_SUPPORT          1       /**< 硬件过流保护支持 */

#define FOC_OCP_THRESHOLD_A         10      /**< 过流保护阈值，单位 A */

#define FOC_DEFAULT_POLE_PAIRS      4       /**< 默认电机极对数 */

#define FOC_DEBUG_ENABLE            1       /**< 调试输出使能 */

/* Experimental routines can energize a stopped motor.  Products must opt in
 * explicitly after providing current, speed, bus-voltage, and timeout limits. */
#ifndef FOC_ENABLE_EXPERIMENTAL_NSD
#define FOC_ENABLE_EXPERIMENTAL_NSD       0   /**< 使能 N/S 极性检测实验模块 */
#endif

#ifndef FOC_ENABLE_EXPERIMENTAL_IDENTIFY
#define FOC_ENABLE_EXPERIMENTAL_IDENTIFY  0   /**< 使能参数辨识实验模块 */
#endif

#ifndef FOC_ENABLE_MOTOR_VERIFY
#define FOC_ENABLE_MOTOR_VERIFY 1           /**< 使能电机验证函数 */
#endif

#ifndef FOC_HF_PROFILE
#define FOC_HF_PROFILE 1                   /**< 使能高频性能分析 */
#endif

#ifndef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 2             /**< 分析等级（0=关, 1=总周期, 2=细分阶段） */
#endif

#if !FOC_HF_PROFILE
#undef FOC_HF_PROFILE_LEVEL
#define FOC_HF_PROFILE_LEVEL 0
#endif

#if (FOC_HF_PROFILE_LEVEL < 0) || (FOC_HF_PROFILE_LEVEL > 2)
#error "FOC_HF_PROFILE_LEVEL must be 0, 1, or 2"
#endif

/* Target-specific DWT timing contract. A zero deadline disables reporting. */
#ifndef FOC_HF_DEADLINE_CYCLES
#define FOC_HF_DEADLINE_CYCLES       0U
#endif

#ifndef FOC_HF_BASELINE_MAX_CYCLES
#define FOC_HF_BASELINE_MAX_CYCLES   0U
#endif

#ifndef FOC_HF_SHADOW_MAX_CYCLES
#define FOC_HF_SHADOW_MAX_CYCLES     0U
#endif

#include "foc_trig.h"

#endif /* __FOC_CONFIG_H__ */
