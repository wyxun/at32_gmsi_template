/****************************************************************************
 * @file    foc_encoder.h
 * @brief   Absolute magnetic-encoder (e.g. AS5600) angle/speed source
 *
 * 编码器输出机械角度/机械速度，电角度由上层按极对数、方向和零位折算。
 *
 * 采样模型：低频任务更新样本缓存（序号递增），20 kHz 步进消费样本；
 * 无新样本时只在电速度达到有效阈值后按速度外插角度。
 * 本模块为纯数学观测器，不包含硬件读取回调或位置源接口。
 ****************************************************************************/

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_angle.h"

typedef struct {
    foc_scalar_t qSpeedFilterAlpha;     /**< 速度低通滤波系数 [0, 1] */
    uint16_t     hwInvalidTimeout;      /**< 无效样本超时，单位 tick */
} foc_encoder_params_t;

typedef struct {
    foc_encoder_params_t tParams;       /**< 参数（Init 后不变） */
    uint8_t      chPolePairs;           /**< 运行时极对数 */
    foc_scalar_t qHighFrequencyPeriod;  /**< 运行时高频周期 (s) */
    foc_angle_t  tMechanicalAngle;      /**< 状态：机械角度 */
    foc_scalar_t qMechanicalSpeed;      /**< 状态：机械速度 (turn/s) */
    uint32_t     wLastSequence;         /**< 状态：上次消费的样本序号 */
    uint16_t     hwLastRawAngle;        /**< 状态：上次 12 位原始角度码 */
    uint16_t     hwTicksSinceSample;    /**< 状态：距上次新样本的 tick 数 */
    bool         bInitialized;          /**< 状态：是否已获得首个样本 */
    bool         bValid;                /**< 状态：输出是否有效 */
} foc_encoder_t;

typedef struct {
    uint16_t hwRawAngle;    /**< 当前 12 位 AS5600 角度码 */
    uint32_t wSequence;     /**< 样本序号；不变表示无新样本 */
    bool     bMagnetOk;     /**< 磁铁位置在正常范围 */
} foc_encoder_sample_t;

typedef struct {
    foc_angle_t  tMechanicalAngle;      /**< 输出机械角度 */
    foc_scalar_t qMechanicalSpeed;      /**< 输出机械速度 (turn/s) */
} foc_encoder_output_t;

/**
 * @brief 用默认参数填充（滤波系数、无效超时）
 * @param ptParams 输出参数
 */
void foc_encoder_DefaultParams(foc_encoder_params_t *ptParams);

/**
 * @brief 初始化编码器观测器实例
 * @param ptEncoder 观测器实例指针
 * @param ptParams  参数指针
 * @return FOC_RESULT_OK 或错误码
 */
foc_result_t foc_encoder_Init(foc_encoder_t *ptEncoder,
                              const foc_encoder_params_t *ptParams,
                              uint8_t chPolePairs,
                              foc_scalar_t qHighFrequencyPeriod);

/**
 * @brief 复位编码器观测器状态（保留参数）
 * @param ptEncoder 观测器实例指针
 */
void foc_encoder_Reset(foc_encoder_t *ptEncoder);

/**
 * @brief 消费一个编码器样本并产生机械角度/速度反馈
 * @param ptEncoder 观测器实例指针
 * @param ptSample  样本输入（原始角度、序号、磁铁状态）
 * @param ptOutput  输出（机械角度、机械速度）
 * @return FOC_RESULT_OK 或样本/安全错误
 */
foc_result_t foc_encoder_Step(foc_encoder_t *ptEncoder,
                              const foc_encoder_sample_t *ptSample,
                              foc_encoder_output_t *ptOutput);

#endif /* FOC_ENCODER_H */
