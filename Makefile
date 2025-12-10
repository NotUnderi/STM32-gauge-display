TARGET := display
BUILD_DIR := build

CC := arm-none-eabi-gcc
AS := $(CC)
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

LD_SCRIPT ?= STM32F407VETX_FLASH.ld
SPECS ?=

MCU := -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
DEFS := -DDEBUG -DLV_LVGL_H_INCLUDE_SIMPLE -DLV_CONF_INCLUDE_SIMPLE -DUSE_HAL_DRIVER -DSTM32F407xx
INCLUDES := \
	-ICore/Inc \
	-IDrivers/lvgl \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Include \
	-IUSB_DEVICE/App \
	-IUSB_DEVICE/Target \
	-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
	-IMiddlewares/ST/STM32_USB_Device_Library/Class/HID/Inc

OPT := -O0
WARN := -Wall
CSTD := -std=gnu11
CYCLO_FLAG ?=

COMMON_FLAGS := $(MCU) -g3 $(DEFS) $(INCLUDES) $(WARN) $(OPT) -ffunction-sections -fdata-sections -fstack-usage $(CYCLO_FLAG) $(SPECS)
CFLAGS := $(COMMON_FLAGS) $(CSTD)
ASFLAGS := $(MCU) -g3 $(DEFS) $(SPECS) -x assembler-with-cpp
LDFLAGS := $(MCU) -T$(LD_SCRIPT) -Wl,-Map=$(TARGET).map -Wl,--gc-sections -static $(SPECS) --specs=nosys.specs -u _printf_float
LDLIBS := -Wl,--start-group -lc -lm -Wl,--end-group

FIND_EXCLUDES := \( -path './Debug' -o -path './build' \) -prune -o
C_SOURCES := $(shell find . $(FIND_EXCLUDES) -type f \( -name '*.c' -o -name '*.C' \) -printf '%P\n')
ASM_SOURCES := $(shell find . $(FIND_EXCLUDES) -type f \( -name '*.s' -o -name '*.S' \) -printf '%P\n')
SRCS := $(C_SOURCES) $(ASM_SOURCES)

OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(patsubst %.C,$(BUILD_DIR)/%.o,$(patsubst %.s,$(BUILD_DIR)/%.o,$(patsubst %.S,$(BUILD_DIR)/%.o,$(SRCS)))))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean hex bin

all: $(TARGET).elf $(TARGET).hex

$(TARGET).elf: $(OBJS) $(LD_SCRIPT)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)
	$(SIZE) $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

bin: $(TARGET).bin
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -MF $(basename $@).d -c $< -o $@

$(BUILD_DIR)/%.o: %.C
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -MF $(basename $@).d -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -MMD -MP -MF $(basename $@).d -c $< -o $@

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -MMD -MP -MF $(basename $@).d -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).map $(DEPS)

-include $(DEPS)
