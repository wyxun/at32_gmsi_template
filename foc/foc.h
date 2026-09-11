/****************************************************************************
 * @file    foc.h
 * @brief   极简 FOC 顶层统一 include
 *          用户只需 #include "foc/foc.h" 即可使用极简单电机 FOC 核心
 ****************************************************************************/

#ifndef __FOC_H__
#define __FOC_H__

#include "foc_config.h"

#include "math/foc_math_types.h"

#include "middleware/foc_core.h"
#include "control/foc_pid.h"
#include "modulation/foc_modulation.h"
#include "observer/foc_encoder.h"
#include "motor/motor.h"

#include "app/foc_app.h"

#endif /* __FOC_H__ */
