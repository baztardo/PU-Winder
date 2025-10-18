cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

set(PICO_BOARD pico CACHE STRING "Board type")
project(motor_test C CXX ASM)

pico_sdk_init()

add_executable(motor_test test.cpp)

target_link_libraries(motor_test
    pico_stdlib
    hardware_gpio
)

pico_enable_stdio_usb(motor_test 1)
pico_enable_stdio_uart(motor_test 0)
pico_add_extra_outputs(motor_test)
