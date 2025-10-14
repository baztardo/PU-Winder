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

# --- Import Pico SDK (adapt the version/path if yours differs) ---
include($ENV{HOME}/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

project(encoder_monitor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(encoder_monitor
    main.c
    encoder.c
)

# PIO sources -> headers
pico_generate_pio_header(encoder_monitor ${CMAKE_CURRENT_LIST_DIR}/quad_encoder.pio)
pico_generate_pio_header(encoder_monitor ${CMAKE_CURRENT_LIST_DIR}/z_index.pio)

target_include_directories(encoder_monitor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(encoder_monitor
    pico_stdlib
    hardware_pio
    hardware_irq
    hardware_timer
    hardware_gpio
)

# USB CDC prints on SKR-Pico
pico_enable_stdio_usb(encoder_monitor 1)
pico_enable_stdio_uart(encoder_monitor 0)

pico_add_extra_outputs(encoder_monitor)