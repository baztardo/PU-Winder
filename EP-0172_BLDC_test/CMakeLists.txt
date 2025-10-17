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

include($ENV{HOME}/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

cmake_minimum_required(VERSION 3.13)

project(bldc_pulse_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()

# Build main executable (with LCD and button UI)
add_executable(bldc_pulse_test
    main.cpp
    bldc_speed_pulse.cpp
    ep7172_lcd.cpp
    button_control.cpp
)

target_include_directories(bldc_pulse_test PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(bldc_pulse_test 
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_spi
)

pico_enable_stdio_usb(bldc_pulse_test 1)
pico_enable_stdio_uart(bldc_pulse_test 0)

pico_add_extra_outputs(bldc_pulse_test)
