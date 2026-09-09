/**
 * @file  as5600.h
 * @brief AS5600 磁旋转编码器驱动（芯片无关，经 mdi_iic_t 访问总线）
 *
 * 使用模型：
 *  - 低频任务（如 1 kHz）调 as5600_Update() 阻塞读取一次原始角度并缓存；
 *  - 高频/ISR 上下文调 as5600_GetSample() 取缓存（无锁、无阻塞）。
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <stdint.h>
#include "mdi/mdi.h"
#include "motor_position.h"

/* AS5600 7 位从机地址（地址绑定在 mdi_iic_t 适配层） */
#define AS5600_I2C_ADDR         0x36U

/* 寄存器（子集） */
#define AS5600_REG_STATUS       0x0BU
#define AS5600_REG_RAW_ANGLE_H  0x0CU
#define AS5600_REG_RAW_ANGLE_L  0x0DU
#define AS5600_REG_ANGLE_H      0x0EU
#define AS5600_REG_MAGNITUDE_H  0x1BU

/* STATUS (0x0B) 位定义 */
#define AS5600_STATUS_MH        (1U << 3)   /**< 磁场过强 */
#define AS5600_STATUS_ML        (1U << 4)   /**< 磁场过弱 */
#define AS5600_STATUS_MD        (1U << 5)   /**< 检测到磁铁 */

#define AS5600_RESOLUTION_BITS  12U
#define AS5600_RESOLUTION       (1U << AS5600_RESOLUTION_BITS)  /* 4096 */

typedef struct {
    uint16_t hwRawAngle;    /**< 12 位原始角度 [0, 4095] */
    uint32_t wSequence;     /**< 样本序号（每成功更新一次递增） */
    uint8_t  chStatus;      /**< STATUS 寄存器原值（MD/ML/MH 位） */
    bool     bMagnetOk;     /**< 磁铁位置在正常范围 */
    bool     bValid;        /**< 样本读取成功 */
} as5600_sample_t;

typedef struct {
    mdi_iic_t      *ptIic;      /**< IIC 总线（pPriv 绑定本机地址） */
    as5600_sample_t tSample;    /**< 最近一次缓存样本 */
} as5600_t;

/**
 * @brief  初始化 AS5600 驱动实例
 * @param  ptThis  实例指针
 * @param  ptIic   IIC 总线接口（pPriv 须绑定 AS5600 地址）
 * @return 0=成功, -1=参数无效
 */
int32_t as5600_Init(as5600_t *ptThis, mdi_iic_t *ptIic);
/**
 * @brief  直接读取 12 位原始角度（阻塞，仅低频上下文调用）
 * @param  ptThis    实例指针
 * @param  phwAngle  输出原始角度 [0, 4095]
 * @return 0=成功, -1=总线错误
 */
int32_t as5600_ReadRawAngle(as5600_t *ptThis, uint16_t *phwAngle);
/**
 * @brief  直接读取 STATUS 寄存器（阻塞，仅低频上下文调用）
 * @param  ptThis     实例指针
 * @param  pchStatus  输出状态字节（MD/ML/MH 位）
 * @return 0=成功, -1=总线错误
 */
int32_t as5600_ReadStatus(as5600_t *ptThis, uint8_t *pchStatus);
/**
 * @brief  判断磁铁是否在正常范围（MD=1 且 MH=0 且 ML=0）
 */
bool    as5600_IsMagnetOk(uint8_t chStatus);
/**
 * @brief  采样一次：读原始角度 + 磁铁状态，写入缓存（1 kHz 任务调用）
 * @param  ptThis  实例指针
 * @return 0=成功, -1=总线错误（缓存保持旧值，bValid 置 false）
 */
int32_t as5600_Update(as5600_t *ptThis);
/**
 * @brief  取缓存样本（ISR 安全，无锁双读校验）
 * @param  ptThis    实例指针
 * @param  ptSample  输出样本
 */
void    as5600_GetSample(const as5600_t *ptThis, as5600_sample_t *ptSample);

#include "foc_encoder.h"

/**
 * @brief AS5600 复合位置传感器（驱动前端 + 外推观测后端）
 */
typedef struct {
    as5600_t      tDriver;   /**< AS5600 硬件驱动 */
    foc_encoder_t tObserver; /**< 20 kHz 拍内外推与滤波观测器 */
    motor_params_t tMotorParams;
    foc_angle_t    tElectricalZero;
    bool           bDirectionInverted;
} as5600_sensor_t;

/**
 * @brief  初始化 AS5600 复合位置传感器
 * @param  ptSensor 复合传感器实例
 * @param  ptIic    I2C 接口
 * @return 0 成功, -1 失败
 */
int32_t as5600_sensor_Init(as5600_sensor_t *ptSensor,
                           mdi_iic_t *ptIic);

/** @brief AS5600 provider ops for the Motor position contract. */
extern const motor_position_ops_t g_tAs5600PositionOps;

#endif /* __AS5600_H__ */
