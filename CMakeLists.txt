cmake_minimum_required(VERSION 3.20)

set(TARGET display)

# Tell CMake this is embedded (so it stops trying to run binaries like an idiot)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Toolchain (assumes it's in PATH)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

project(${TARGET} C ASM)

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VETX_FLASH.ld)

# Source files
file(GLOB_RECURSE C_SOURCES
    ${CMAKE_SOURCE_DIR}/*.c
)

file(GLOB_RECURSE ASM_SOURCES
    ${CMAKE_SOURCE_DIR}/*.s
    ${CMAKE_SOURCE_DIR}/*.S
)

# Exclude build dirs
list(FILTER C_SOURCES EXCLUDE REGEX ".*/(build|Debug)/.*")
list(FILTER ASM_SOURCES EXCLUDE REGEX ".*/(build|Debug)/.*")

# Target
add_executable(${TARGET}.elf
    ${C_SOURCES}
    ${ASM_SOURCES}
)

# Include directories (scoped properly)
target_include_directories(${TARGET}.elf PRIVATE
    Core/Inc
    Drivers/lvgl/src
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    USB_DEVICE/App
    USB_DEVICE/Target
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc
)

# Defines
target_compile_definitions(${TARGET}.elf PRIVATE
    DEBUG
    LV_LVGL_H_INCLUDE_SIMPLE
    LV_CONF_INCLUDE_SIMPLE
    USE_HAL_DRIVER
    STM32F407xx
)

# Compile flags (correct way)
target_compile_options(${TARGET}.elf PRIVATE
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -g3
    -O0
    -Wall
    -ffunction-sections
    -fdata-sections
    -fstack-usage
    -std=gnu11
)

# ASM-specific flags
target_compile_options(${TARGET}.elf PRIVATE
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=cortex-m4
        -mthumb
        -g3
    >
)

# Linker flags (correct way)
target_link_options(${TARGET}.elf PRIVATE
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -T${LINKER_SCRIPT}
    -Wl,-Map=${TARGET}.map
    -Wl,--gc-sections
    -static
    --specs=nosys.specs
    -u _printf_float
)

# Libraries
target_link_libraries(${TARGET}.elf PRIVATE
    c
    m
)

# Post-build: size
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}.elf>
)

# Post-build: HEX
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${TARGET}.elf>
            ${TARGET}.hex
)

# Post-build: BIN
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${TARGET}.elf>
            ${TARGET}.bin
)

# Clean extra files
set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES
    "${TARGET}.hex;${TARGET}.bin;${TARGET}.map"
)
