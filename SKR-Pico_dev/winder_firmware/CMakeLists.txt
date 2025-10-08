cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(winder_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(winder_firmware
    main.cpp
    tmc2209.cpp
    encoder.cpp
    stepcompress.cpp
    move_queue.cpp
    scheduler.cpp
    lcd_display.cpp
    winding_controller.cpp
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(winder_firmware)

# Link libraries
target_link_libraries(winder_firmware
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_uart
    hardware_timer
    hardware_i2c
    hardware_dma
    hardware_pwm
    hardware_irq
)

# Enable USB output, disable UART output (or vice versa)
pico_enable_stdio_usb(winder_firmware 1)
pico_enable_stdio_uart(winder_firmware 0)

# Compiler optimizations
target_compile_options(winder_firmware PRIVATE
    -Wall
    -Wextra
    -O2
    -ffunction-sections
    -fdata-sections
)

# Linker optimizations
target_link_options(winder_firmware PRIVATE
    -Wl,--gc-sections
)