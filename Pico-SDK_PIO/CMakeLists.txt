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

include($ENV{HOME}/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

project(encoder_monitor C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Add executable FIRST - before pico_generate_pio_header
add_executable(encoder_monitor
    main.c
    src/i2c/i2c_helper.c
    src/drivers/lcd_pcf8574.c
    src/encoder/encoder.c
)

# THEN generate PIO header - after add_executable
pico_generate_pio_header(encoder_monitor 
    ${CMAKE_CURRENT_LIST_DIR}/src/encoder/encoder.pio
)

target_include_directories(encoder_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/encoder
)

target_link_libraries(encoder_monitor
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_gpio
    hardware_i2c
    hardware_timer
)

pico_enable_stdio_usb(encoder_monitor 1)
pico_enable_stdio_uart(encoder_monitor 0)

pico_add_extra_outputs(encoder_monitor)