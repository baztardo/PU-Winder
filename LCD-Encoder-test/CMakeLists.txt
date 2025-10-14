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
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
# --- Import Pico SDK ---
include($ENV{HOME}/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

# Project name
project(encoder_monitor C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(encoder_monitor
    main.c
    src/i2c/i2c_helper.c
    src/drivers/lcd_pcf8574.c
    src/encoder/encoder.c
)

# Add include directories
target_include_directories(encoder_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/encoder
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(encoder_monitor)

# Link libraries
target_link_libraries(encoder_monitor
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_timer
)

# Enable USB serial output (disable UART)
pico_enable_stdio_usb(encoder_monitor 1)
pico_enable_stdio_uart(encoder_monitor 0)

pico_add_extra_outputs(encoder_monitor)
