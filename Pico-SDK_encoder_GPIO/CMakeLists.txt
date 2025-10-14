# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# --- Import Pico SDK ---
include($ENV{HOME}/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

# Project name
project(encoder_monitor C CXX ASM)

# C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Init SDK
pico_sdk_init()

# Add executable with all source files
add_executable(encoder_monitor
    main.c
    i2c_helper.c
    lcd_pcf8574.c
    encoder_gpio.c    # NEW: GPIO encoder instead of PIO
)

# REMOVED: No need for PIO header generation anymore
# pico_generate_pio_header() line is deleted

# Include directories
target_include_directories(encoder_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link libraries
target_link_libraries(encoder_monitor
    pico_stdlib
    hardware_gpio      # GPIO interrupts
    hardware_i2c
    hardware_timer

)

# STDIO: USB on, UART off
pico_enable_stdio_usb(encoder_monitor 1)
pico_enable_stdio_uart(encoder_monitor 0)

# Produce UF2/BIN/HEX
pico_add_extra_outputs(encoder_monitor)
