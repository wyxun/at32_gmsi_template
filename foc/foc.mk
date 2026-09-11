# =============================================================================
# Modus FOC — source and include configuration (minimal single-motor build)
#
# Include this from target/<chip>/target.mk for chips that support FOC.
# Chips that do not support FOC skip the include; FOC_SOURCES stays empty
# (guarded by "FOC_SOURCES ?=" in the top-level makefile) and the FOC
# include paths default to nothing.
#
# 极简构建：只编译纯数学核心 + PID + SVPWM + 编码器 + 应用 + 三角函数。
# 旧多实例框架（motor/、foc_hal*.c 等）不再进入构建；未参与构建的高级
# 算法源码保留在仓库作参考（计划 §8），不进入 FOC_SOURCES。
# =============================================================================

FOC_INCLUDES = -Ifoc \
               -Ifoc/math -Ifoc/hal -Ifoc/motor \
               -Ifoc/middleware -Ifoc/control \
               -Ifoc/modulation -Ifoc/observer \
               -Ifoc/app

FOC_SOURCES = foc/math/foc_numeric.c \
              foc/math/foc_angle.c \
              foc/math/foc_trig_lut.c \
              foc/math/foc_math.c \
              foc/middleware/foc_core.c \
              foc/control/foc_pid.c \
              foc/modulation/foc_modulation.c \
              foc/observer/foc_encoder.c \
              foc/motor/motor.c \
              foc/app/foc_app.c
