# ==============================================================================
# Project Makefile — Multi-chip support via target/$(TARGET_CHIP)/
# Toolchain : LLVM Embedded Toolchain for Arm (Windows)
# Usage     : make [BUILD=debug|release] [TARGET_CHIP=stm32g431] ...
# ==============================================================================
SW_ROOT ?= D:/software
MSYS2_BIN = $(SW_ROOT)/msys64/mingw64/bin
MAKE      = $(MSYS2_BIN)/mingw32-make.exe

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
TARGET = template

# Support both target=xxx (lowercase) and TARGET_CHIP=xxx (uppercase)
ifdef target
    TARGET_CHIP = $(target)
endif

TARGET_CHIP ?= stm32g431
CLASS_SOURCES = class/template_class.c

include target/$(TARGET_CHIP)/target.mk

# ------------------------------------------------------------------------------
# Toolchain  (Windows LLVM path)
# ------------------------------------------------------------------------------
ifeq ($(TARGET_CHIP),ch592)
    TARGET_TRIPLE = --target=riscv32-none-elf
    LLVM_PATH = $(SW_ROOT)/msys64/mingw64/bin/
else
    TARGET_TRIPLE = --target=armv7em-none-eabi
    LLVM_PATH = $(SW_ROOT)/llvm_for_arm/bin/
endif

# Always use llvm_for_arm's multi-architecture binary utilities
LLVM_UTILS_PATH = $(SW_ROOT)/llvm_for_arm/bin/

CC  = $(LLVM_PATH)clang
AS  = $(LLVM_PATH)clang
CP  = $(LLVM_UTILS_PATH)llvm-objcopy
SZ  = $(LLVM_UTILS_PATH)llvm-size
LD  = $(LLVM_PATH)clang
AR  = $(LLVM_UTILS_PATH)llvm-ar
NM  = $(LLVM_UTILS_PATH)llvm-nm

# ------------------------------------------------------------------------------
# Directories
# ------------------------------------------------------------------------------
BUILD_DIR = build

ifeq ($(OS),Windows_NT)
    SHELL = cmd.exe
    MKDIR = if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)
    RMDIR = if exist $(BUILD_DIR) rd /s /q $(BUILD_DIR)
else
    MKDIR = mkdir -p $(BUILD_DIR)
    RMDIR = rm -rf $(BUILD_DIR)
endif

# ------------------------------------------------------------------------------
# MODUS framework
# ------------------------------------------------------------------------------
MODUS_ROOT = modus

# Build mode: BUILD=debug | BUILD=debug-rel (default) | BUILD=release
BUILD ?= debug-rel

MODUS_ENABLE ?= 1

# Module switches — default for debug; overridden by release below
MSHELL_ENABLE    = 1
MWAVEFORM_ENABLE = 1
MWAVEFORM_RTT_BUFFER_SIZE = 4096
MSTORAGE_ENABLE  = 1
MBLINFO_ENABLE   = 1
MODUS_USE_LOG    = 1

# Build mode overrides (MUST be before modus.mk include)
ifeq ($(BUILD),release)
    # Production: no debug modules, completely compiled-out MODUS
    OPT = -Os
    MODUS_ENABLE = 1
    MSHELL_ENABLE    = 0
    MWAVEFORM_ENABLE = 0
    MSTORAGE_ENABLE  = 0
    MBLINFO_ENABLE   = 0
    MODUS_USE_LOG    = 0
    C_DEFS += -DFOC_HF_PROFILE=0 -DFOC_HF_PROFILE_LEVEL=0
else ifeq ($(BUILD),debug-rel)
    # Release-close debugging: same -Os, but debug modules on
    OPT = -Os
else
    # Daily development: no optimization
    OPT = -O0
endif

# 默认关闭 MODUS 内置移植，各芯片 target.mk 可自行覆盖启用
MODUS_USE_DEFAULT_PERFC_PORT ?= 0

ifeq ($(MODUS_ENABLE),1)
    include $(MODUS_ROOT)/modus.mk
    C_DEFS += -DMODUS_ENABLE=1
else
    C_DEFS += -DMODUS_ENABLE=0 -D__NO_USE_LOG__ -D__NO_USE_ASSERT
    MODUS_INCLUDES = \
        -I$(MODUS_ROOT) \
        -I$(MODUS_ROOT)/src \
        -I$(MODUS_ROOT)/src/mdi \
        -I$(MODUS_ROOT)/src/arch \
        -I$(MODUS_ROOT)/src/arch/cortex-m \
        -I$(MODUS_ROOT)/src/arch/riscv \
        -I$(LIB_PLOOC_DIR) \
        -I$(LIB_PERF_DIR)
    MODUS_CFLAGS = -DMSHELL_ENABLE=0 -DMWAVEFORM_ENABLE=0 -D__NO_USE_LOG__ -D__NO_USE_ASSERT
    MODUS_SRCS = \
        $(MODUS_ROOT)/src/arch/perfc_port.c \
        $(MODUS_ROOT)/src/utilities/mringbuf.c
endif

# grblHAL CNC controller (opt-in per target via GRBLHAL_ENABLE=1)
ifdef GRBLHAL_ENABLE
GRBLHAL_ROOT = third_party/grblhal
include $(GRBLHAL_ROOT)/grblhal.mk
endif

LIB_PERF_DIR  = $(MODUS_ROOT)/lib/perf_counter
LIB_PLOOC_DIR = $(MODUS_ROOT)/lib/plooc

# MODUS sources are automatically handled via MODUS_SRCS from modus.mk
PERIF_LIB_SOURCES = \
    $(LIB_PERF_DIR)/perf_counter.c

PERIPHERAL_SOURCES = $(wildcard peripheral/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/$(TARGET_CHIP)/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/driver/*.c)


# FOC framework — defined by foc/foc.mk (included from target.mk).
# Targets without FOC support leave both empty.
FOC_SOURCES ?=
FOC_INCLUDES ?=

# FOC numeric backend is selected explicitly, never inferred from the CPU.
FOC_NUMERIC ?= float
ifeq ($(FOC_NUMERIC),float)
    C_DEFS += -DFOC_NUMERIC_FLOAT=1
else ifeq ($(FOC_NUMERIC),fixed)
    C_DEFS += -DFOC_NUMERIC_FIXED=1
else
    $(error FOC_NUMERIC must be float or fixed)
endif

# Motor-energizing experiments are opt-in for product builds.
FOC_EXPERIMENTAL_NSD ?= 0
FOC_EXPERIMENTAL_IDENTIFY ?= 0
C_DEFS += -DFOC_ENABLE_EXPERIMENTAL_NSD=$(FOC_EXPERIMENTAL_NSD)
C_DEFS += -DFOC_ENABLE_EXPERIMENTAL_IDENTIFY=$(FOC_EXPERIMENTAL_IDENTIFY)

# ------------------------------------------------------------------------------
# All C sources  (chip-specific vars set by target.mk)
# ------------------------------------------------------------------------------
C_SOURCES = \
    src/main.c \
    $(PERFC_PORT_C) \
    $(IT_C) \
    $(SYSTEM_C) \
    $(CHIP_SOURCES) \
    $(MODUS_SRCS) \
    $(PERIF_LIB_SOURCES) \
    $(PERIPHERAL_SOURCES) \
    $(FOC_SOURCES)

ifeq ($(MODUS_ENABLE),1)
    C_SOURCES += $(CLASS_SOURCES)
endif

# grblHAL sources (only when enabled)
ifdef GRBLHAL_ENABLE
C_SOURCES += $(GRBLHAL_SRCS)
C_SOURCES += $(wildcard grblhal_adapt/*.c)
endif

ASM_SOURCES = $(STARTUP_S)

# ------------------------------------------------------------------------------
# Defines
# ------------------------------------------------------------------------------
C_DEFS += \
    -DMSHELL_MAX_CMDS=32 \
    -D__PERFC_USE_USER_CUSTOM_PORTING__=1 \
    -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1 \
    -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port.h\" \
    -D__COMPILER_HAS_GNU_EXTENSIONS__=1 \
    -DTRACE_USE_LIBC_PRINTF=0 \
    -DTRACE_MCU_WRITE_STRING="extern void user_trace_output(const char*); user_trace_output" \
    -DMODUS_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""

# ------------------------------------------------------------------------------
# Include paths
# ------------------------------------------------------------------------------
C_INCLUDES = \
    -I. \
    -Isrc \
    -I$(CMSIS_CORE) \
    -I$(CMSIS_DEV) \
    $(TARGET_INCLUDES) \
    $(MODUS_INCLUDES) \
    -I$(MODUS_ROOT)/src/utilities \
    -I$(MODUS_ROOT)/src/mdebug \
    -I$(MODUS_ROOT)/src/mdebug/segger_rtt \
    -I$(LIB_PLOOC_DIR) \
    -I$(LIB_PERF_DIR) \
    -Iperipheral \
    -Iperipheral/$(TARGET_CHIP) \
    -Iperipheral/driver \
    -Iclass \
    $(FOC_INCLUDES)

# grblHAL includes and defines (only when enabled, after C_INCLUDES is fully built)
ifdef GRBLHAL_ENABLE
C_INCLUDES += $(GRBLHAL_INCLUDES)
C_INCLUDES += -Igrblhal_adapt
C_DEFS     += $(GRBLHAL_CFLAGS)
endif

# ------------------------------------------------------------------------------
# Flags
# ------------------------------------------------------------------------------
$(info C_DEFS is $(C_DEFS))
CFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS) $(C_DEFS) $(C_INCLUDES) $(MODUS_CFLAGS) $(OPT) -g
CFLAGS += -Wall -Wextra -std=gnu11
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-exceptions
CFLAGS += -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare \
          -Wno-compare-distinct-pointer-types -Wno-unused-command-line-argument

ASFLAGS = $(TARGET_TRIPLE) $(CPU_FLAGS) -g

LDFLAGS += $(TARGET_TRIPLE) $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
STD_LIBS ?= -lcrt0 -lc -lm
LDFLAGS += $(STD_LIBS)

# ------------------------------------------------------------------------------
.PHONY: all clean size flash rtt debug-rel
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex size

release:
	$(MAKE) BUILD=release

debug-rel:
	$(MAKE) BUILD=debug-rel

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
ASM_OBJECTS = $(patsubst %.s,$(BUILD_DIR)/%.o,$(patsubst %.S,$(BUILD_DIR)/%.o,$(notdir $(ASM_SOURCES))))
OBJECTS += $(ASM_OBJECTS)

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

# 针对高频控制步相关的密集计算模块强制启用 -O2 优化
$(BUILD_DIR)/foc_%.o: CFLAGS += -O2
$(BUILD_DIR)/motor.o: CFLAGS += -O2

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -R .ARM.attributes $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex -R .ARM.attributes -R ._user_heap_stack $< $@

$(BUILD_DIR):
	$(MKDIR)

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

clean:
	$(RMDIR)

# ------------------------------------------------------------------------------
# Flash / Debug
# ------------------------------------------------------------------------------

ifeq ($(OS),Windows_NT)
    OPENOCD_BIN     ?= $(SW_ROOT)/msys64/mingw64/bin/openocd.exe
    OPENOCD_SCRIPTS ?= $(SW_ROOT)/msys64/mingw64/share/openocd/scripts
    OPENOCD_CMD ?= $(OPENOCD_BIN) -s $(OPENOCD_SCRIPTS) -f target/$(TARGET_CHIP)/openocd.cfg -c "adapter speed 4000" -c "tcl_port 0"
else
    OPENOCD_BIN = openocd
    OPENOCD_CMD = $(OPENOCD_BIN) -f target/$(TARGET_CHIP)/openocd.cfg
endif

FLASH_CMD ?= $(OPENOCD_CMD) -c "init" -c "reset halt" -c "sleep 200" -c "program $< verify" -c "reset run" -c "exit"

flash: $(BUILD_DIR)/$(TARGET).hex
	$(FLASH_CMD)

download:
	$(OPENOCD_CMD) -c "init" -c "reset halt" -c "sleep 200" -c "program $(BUILD_DIR)/$(TARGET).hex verify" -c "reset run" -c "exit"

debug-server:

	$(OPENOCD_CMD)

openocd: debug-server

debug: $(BUILD_DIR)/$(TARGET).elf
	gdb-multiarch -x debug.gdb $<

# ------------------------------------------------------------------------------
# RTT
# ------------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
    RTT_ADDR = $(shell powershell -NoProfile -Command "$$nm = & '$(NM).exe' $(BUILD_DIR)/$(TARGET).elf 2>$$null | Select-String '_SEGGER_RTT$$'; if ($$nm) { '0x' + ($$nm -split ' ')[0] }" 2>nul)
else
    RTT_ADDR = $(shell $(NM) $(BUILD_DIR)/$(TARGET).elf 2>/dev/null | awk '/_SEGGER_RTT$$/ {print "0x"$$1}')
endif

# 1 ms polling is required for the 20 kHz waveform path. The 10 ms default
# measured ~220 batch frames/s (~7k samples/s), while 1 ms measured ~300
# 64-sample batch frames/s (~19k samples/s) on STM32G431 + CMSIS-DAP.
RTT_CMD ?= -c "init" -c "catch { resume }" -c "rtt setup $(RTT_ADDR) 0xa8 \"SEGGER RTT\"" -c "rtt start" -c "rtt polling_interval 1" -c "rtt server start 9090 0" -c "rtt server start 9091 1"

rtt-addr: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"

rtt: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"
	$(OPENOCD_CMD) $(RTT_CMD)

# Flash via already-running OpenOCD telnet (port 4444)
flash-rtt: $(BUILD_DIR)/$(TARGET).hex
ifeq ($(OS),Windows_NT)
	@powershell -NoProfile -Command \
	    "$$hex = (Resolve-Path '$<').Path; \
	     $$c = New-Object System.Net.Sockets.TcpClient('localhost', 4444); \
	     $$s = $$c.GetStream(); \
	     $$w = New-Object System.IO.StreamWriter($$s); \
	     $$w.WriteLine(\"program $$hex verify reset\"); $$w.WriteLine('exit'); \
	     $$w.Flush(); Start-Sleep 2; $$c.Close()"
else
	@ABS=$$(realpath $<); \
	printf "program $$ABS verify reset\nexit\n" | nc -w 10 localhost 4444; \
	sleep 1; \
	printf "rtt stop\nrtt start\n" | nc -w 3 localhost 4444
endif

# ------------------------------------------------------------------------------
# Info
# ------------------------------------------------------------------------------
info:
	@echo "TARGET      = $(TARGET)"
	@echo "TARGET_CHIP = $(TARGET_CHIP)"
	@echo "BUILD_DIR   = $(BUILD_DIR)"
	@echo "LDSCRIPT    = $(LDSCRIPT)"
	@echo "LLVM_PATH   = $(LLVM_PATH)"
	@echo "C_SOURCES   = $(C_SOURCES)"
	@echo "ASM_SOURCES = $(ASM_SOURCES)"

.PHONY: all release debug-rel clean flash download flash-rtt debug-server debug size info rtt rtt-addr openocd
