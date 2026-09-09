CC ?= clang
CFLAGS = -std=gnu11 -Wall -Wextra -Werror -I. -I../../foc -I../../foc/math \
         -I../../foc/middleware -I../../foc/modulation -I../../foc/control \
         -I../../foc/hal -I../../foc/observer \
         -I../../foc/app -I../../foc/experimental -I../../foc/motor
CFLAGS += -I../../foc/optimization
CFLAGS += -I../../peripheral/driver -I../../peripheral/stm32g431 \
          -I../../modus/src -I../../modus/src/mdi -I../../modus/src/mdebug \
          -I../../modus/src/utilities
CFLAGS += -I../../modus/lib/perf_counter \
          -I../../modus/lib/plooc \
          -Wno-cpp \
          -D__PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__=1 \
          -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port_stub.h\" \
          -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1 \
          -D__NO_USE_LOG__ -D__NO_USE_ASSERT -DFOC_APP_TEST
STRICT_ALIAS_CFLAGS ?=
EXTRA_CFLAGS ?=
CFLAGS += $(STRICT_ALIAS_CFLAGS) $(EXTRA_CFLAGS)

.PHONY: all minimal minimal-core minimal-core-float minimal-core-fixed \
        minimal-encoder minimal-encoder-float minimal-encoder-fixed \
        minimal-lifecycle minimal-lifecycle-float minimal-lifecycle-fixed \
        motor-float motor-fixed as5600-float as5600-fixed \
        framework-contract framework-contract-float framework-contract-fixed \
        clean

MINIMAL_CORE_SRCS = test_foc_minimal_core.c \
                    ../../foc/math/foc_numeric.c \
                    ../../foc/math/foc_angle.c \
                    ../../foc/math/foc_trig_lut.c \
                    ../../foc/math/foc_math.c \
                    ../../foc/middleware/foc_core.c \
                    ../../foc/control/foc_pid.c \
                    ../../foc/modulation/foc_modulation.c

minimal-core: minimal-core-float minimal-core-fixed

minimal-core-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_CORE_SRCS) \
	    -lm -o minimal_core_float
	./minimal_core_float

minimal-core-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_CORE_SRCS) \
	    -lm -o minimal_core_fixed
	./minimal_core_fixed

MINIMAL_ENCODER_SRCS = test_foc_minimal_encoder.c \
                       ../../foc/math/foc_numeric.c \
                       ../../foc/math/foc_angle.c \
                       ../../foc/math/foc_trig_lut.c \
                       ../../foc/math/foc_math.c \
                       ../../foc/observer/foc_encoder.c

minimal-encoder: minimal-encoder-float minimal-encoder-fixed

minimal-encoder-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_ENCODER_SRCS) \
	    -lm -o minimal_encoder_float
	./minimal_encoder_float

minimal-encoder-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_ENCODER_SRCS) \
	    -lm -o minimal_encoder_fixed
	./minimal_encoder_fixed

MINIMAL_LIFECYCLE_SRCS = test_foc_minimal_lifecycle.c \
                         ../../foc/math/foc_numeric.c \
                         ../../foc/math/foc_angle.c \
                         ../../foc/math/foc_trig_lut.c \
                         ../../foc/math/foc_math.c \
                         ../../foc/middleware/foc_core.c \
                         ../../foc/control/foc_pid.c \
                         ../../foc/modulation/foc_modulation.c \
                         ../../foc/observer/foc_encoder.c \
                         ../../foc/motor/motor.c \
                         ../../foc/app/foc_app.c

minimal-lifecycle: minimal-lifecycle-float minimal-lifecycle-fixed

minimal-lifecycle-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(MINIMAL_LIFECYCLE_SRCS) \
	    -lm -o minimal_lifecycle_float
	./minimal_lifecycle_float

minimal-lifecycle-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(MINIMAL_LIFECYCLE_SRCS) \
	    -lm -o minimal_lifecycle_fixed
	./minimal_lifecycle_fixed

MOTOR_SRCS = test_motor.c \
             ../../foc/math/foc_numeric.c \
             ../../foc/math/foc_angle.c \
             ../../foc/math/foc_trig_lut.c \
             ../../foc/math/foc_math.c \
             ../../foc/middleware/foc_core.c \
             ../../foc/control/foc_pid.c \
             ../../foc/modulation/foc_modulation.c \
             ../../foc/motor/motor.c

motor-float:
	$(CC) $(CFLAGS) -DPERFC_TEST_CRITICAL_HOOKS=1 \
	    -DFOC_NUMERIC_FLOAT=1 $(MOTOR_SRCS) \
	    -lm -o motor_float
	./motor_float

motor-fixed:
	$(CC) $(CFLAGS) -DPERFC_TEST_CRITICAL_HOOKS=1 \
	    -DFOC_NUMERIC_FIXED=1 $(MOTOR_SRCS) \
	    -lm -o motor_fixed
	./motor_fixed

FRAMEWORK_CONTRACT_SRCS = test_framework_contract.c \
                          ../../foc/math/foc_numeric.c \
                          ../../foc/math/foc_angle.c \
                          ../../foc/math/foc_trig_lut.c \
                          ../../foc/math/foc_math.c \
                          ../../foc/middleware/foc_core.c \
                          ../../foc/control/foc_pid.c \
                          ../../foc/modulation/foc_modulation.c \
                          ../../foc/motor/motor.c

framework-contract: framework-contract-float framework-contract-fixed

framework-contract-float:
	$(CC) $(CFLAGS) -DPERFC_TEST_CRITICAL_HOOKS=1 \
	    -DFOC_NUMERIC_FLOAT=1 $(FRAMEWORK_CONTRACT_SRCS) \
	    -lm -o framework_contract_float
	./framework_contract_float

framework-contract-fixed:
	$(CC) $(CFLAGS) -DPERFC_TEST_CRITICAL_HOOKS=1 \
	    -DFOC_NUMERIC_FIXED=1 $(FRAMEWORK_CONTRACT_SRCS) \
	    -lm -o framework_contract_fixed
	./framework_contract_fixed

AS5600_SRCS = test_as5600.c \
              ../../foc/math/foc_numeric.c \
              ../../foc/math/foc_angle.c \
              ../../foc/math/foc_trig_lut.c \
              ../../foc/math/foc_math.c \
              ../../foc/observer/foc_encoder.c \
              ../../peripheral/driver/as5600.c

as5600-float:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FLOAT=1 $(AS5600_SRCS) \
	    -lm -o as5600_float
	./as5600_float

as5600-fixed:
	$(CC) $(CFLAGS) -DFOC_NUMERIC_FIXED=1 $(AS5600_SRCS) \
	    -lm -o as5600_fixed
	./as5600_fixed

minimal: minimal-core minimal-encoder minimal-lifecycle \
         motor-float motor-fixed as5600-float as5600-fixed \
         framework-contract

all: minimal

clean:
	$(RM) minimal_core_float minimal_core_fixed \
	      minimal_encoder_float minimal_encoder_fixed \
	      minimal_lifecycle_float minimal_lifecycle_fixed \
      motor_float motor_fixed as5600_float as5600_fixed \
      framework_contract_float framework_contract_fixed
