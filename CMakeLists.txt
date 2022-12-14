cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(mapple_pio_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(
        -O3
        )

add_executable(maple_pio)

target_compile_definitions(maple_pio PRIVATE PICO_HW)

pico_enable_stdio_usb(maple_pio 1)
pico_add_extra_outputs(maple_pio)

pico_generate_pio_header(maple_pio ${CMAKE_CURRENT_LIST_DIR}/src/maple.pio)

target_sources(maple_pio PRIVATE src/maple.c src/state_machine.c )

target_link_libraries(maple_pio PRIVATE
        pico_stdlib
        pico_multicore
        pico_time
        hardware_adc
        hardware_pio
        hardware_dma
        hardware_pwm
        hardware_timer
        hardware_i2c
        hardware_spi
        hardware_flash
        hardware_vreg
        )



