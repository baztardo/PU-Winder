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

project(winder_project C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable
add_executable(winder_project
    main.cpp
    src/tmc2209.cpp
    src/encoder.cpp
    src/stepcompress.cpp
    src/move_queue.cpp
    src/scheduler.cpp
    src/lcd_display.cpp
    src/winding_controller.cpp
)
target_include_directories(winder_project PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Create map/bin/hex/uf2 files
#pico_add_extra_outputs(winder_project)

# Link libraries
target_link_libraries(winder_project
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
pico_enable_stdio_usb(winder_project 1)
pico_enable_stdio_uart(winder_project 0)

# Enable USB serial and stdio
pico_enable_stdio_usb(winder_project 1)
pico_enable_stdio_uart(winder_project 0)

pico_add_extra_outputs(winder_project)

