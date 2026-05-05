cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "/home/lidar_cart/pico/pico-sdk" CACHE PATH "Pico SDK path" FORCE)
set(PICOTOOL_FETCH_FROM_GIT_PATH "/home/lidar_cart/builds/picotool" CACHE PATH "" FORCE)
set(PICO_BOARD pico2 CACHE STRING "Board type" FORCE)

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(encoder_imu_c C CXX ASM)

pico_sdk_init()

set(SRC_FILES
    src/main.cpp
    src/QuadratureDecoder.cpp
)

add_executable(encoder_imu_c
    ${SRC_FILES}
)

pico_generate_pio_header(
    encoder_imu_c
    ${CMAKE_CURRENT_LIST_DIR}/src/QuadratureDecoder.pio
)

pico_generate_pio_header(
    encoder_imu_c
    ${CMAKE_CURRENT_LIST_DIR}/src/pps_capture.pio
)

target_include_directories(encoder_imu_c PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(encoder_imu_c
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_i2c
    hardware_pio
    hardware_dma
    hardware_irq
)

pico_enable_stdio_usb(encoder_imu_c 1)
pico_enable_stdio_uart(encoder_imu_c 0)

pico_add_extra_outputs(encoder_imu_c)