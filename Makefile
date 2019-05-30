######################################
# target
######################################
TARGET = hover

######################################
# building variables
######################################
# debug build?
DEBUG = 0
# optimization
OPT = -Og

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_adc.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_bkp.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_can.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_crc.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_dac.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_dbgmcu.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_dma.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_exti.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_flash.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_gpio.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_i2c.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_iwdg.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_pwr.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_rcc.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_rtc.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_sdio.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_spi.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_tim.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_usart.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_wwdg.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/at32f4xx_xmc.c \
Drivers/AT32F4xx_StdPeriph_Driver/Src/misc.c \
Src/system_at32f4xx.c \
Src/main.c \
Src/at32f4xx_it.c \

# ASM sources
ASM_SOURCES =  \
startup_at32f403xe.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi
#from https://github.com/antoinealb/rust-demo-cortex-m4/blob/master/Makefile
#FPU=-mfpu=fpv4-sp-d16 
FPU=-mfpu=fpv4-sp-d16
#FLOAT-ABI=-mfloat-abi=softfp
FLOAT-ABI=-mfloat-abi=soft


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DAT32F403Rx_HD
#-DAT32F403xE \
#-DUSE_HAL_DRIVER \

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/AT32F4xx_StdPeriph_Driver/Inc \
-IDrivers/CMSIS/CM4/DeviceSupport \
-IDrivers/CMSIS/CM4/CoreSupport \


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=c99

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = AT32F403RCTx_FLASH.ld

# libraries
#Drivers/CMSIS/Lib/GCC
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ -include Inc/at32f4xx_conf.h

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

format:
	find Src/ Inc/ -iname '*.h' -o -iname '*.c' | xargs clang-format -i
#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

flash-jlink:
	 JLink.exe -if swd -device Cortex-M4 -speed 4000 -SettingsFile .\JLinkSettings.ini -CommanderScript jlink-command.jlink
	
unlock:
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0"

#debug: 
#	JLinkGDBServer -select USB -device AT32F403RCT6 -endian little -if SWD -speed 4000 -noir -LocalhostOnly
#	gdb build/hover.elf

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
