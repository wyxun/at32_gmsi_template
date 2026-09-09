# =============================================================================
# target/at32f413/target.mk
# AT32F413RCT7 — Cortex-M4F, single-precision FPU (Motor EVB V1)
# =============================================================================

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DAT32F413RCT7 -DUSE_STDPERIPH_DRIVER -DFOC_SUPPORT=1 -DAT_MOTOR_EVB_V1 -DCORE_DEBUG_OVERRIDE_FAULT_HANDLER
C_DEFS += -DFOC_PORT_HAS_POSITION=0

# CMSIS / peripheral library paths
CHIPLIB_ROOT = vendor/cortex-m/AT32F413_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/at32f413/AT32F413xC_FLASH.ld
STARTUP_S    = $(CMSIS_DEV)/startup/gcc/startup_at32f413.s
SYSTEM_C     = $(CMSIS_DEV)/system_at32f413.c
IT_C         = target/at32f413/at32f413_it.c


# Chip-specific include paths
TARGET_INCLUDES = -I$(DRV_INC) -Itarget/at32f413

# ------------------------------------------------------------------
# AT32F413 peripheral driver selection
# ------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_TMR    ?= 1
USE_DRV_USART  ?= 1
USE_DRV_ADC    ?= 1
USE_DRV_DMA    ?= 1
USE_DRV_DEBUG  ?= 1
USE_DRV_EXINT  ?= 0
USE_DRV_SPI    ?= 0
USE_DRV_I2C    ?= 0
USE_DRV_CAN    ?= 0
USE_DRV_WDT    ?= 0
USE_DRV_WWDT   ?= 0
USE_DRV_PWC    ?= 0
USE_DRV_RTC    ?= 0
USE_DRV_BPR    ?= 0
USE_DRV_CRC    ?= 0
USE_DRV_SDIO   ?= 0
USE_DRV_USB    ?= 0
USE_DRV_ACC    ?= 0

DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_flash.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_tmr.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_usart.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_adc.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_dma.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_debug.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_exint.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_i2c.c
endif
ifeq ($(USE_DRV_CAN),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_can.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_wwdt.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_pwc.c
endif

CHIP_SOURCES = $(DRV_SOURCES)

# FOC framework — AT32F413 supports FOC
include foc/foc.mk

# OpenOCD (AT32 custom v0.11 — same as at32f421)
OPENOCD_BIN     = $(SW_ROOT)/msys64/mingw64/bin/openocd-at32.exe
OPENOCD_SCRIPTS = $(SW_ROOT)/msys64/mingw64/share/openocd/scripts

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1
