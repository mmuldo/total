# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_PATH "/YOUR_PICO_SDK_PATH/pico-sdk")

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(conductivity C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

# Initialise the Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(conductivity)
target_sources(conductivity PRIVATE conductivity.c i2c.c)

pico_set_program_name(conductivity "conductivity")
pico_set_program_version(conductivity "0.1")

pico_enable_stdio_uart(conductivity 0)
pico_enable_stdio_usb(conductivity 1)

# Add the standard library to the build
target_link_libraries(conductivity pico_stdlib pico_multicore)

# Add any user requested libraries
target_link_libraries(conductivity
        hardware_pio
        hardware_timer
        hardware_clocks
        hardware_pwm
        hardware_adc
        hardware_dma
        hardware_watchdog
        hardware_i2c
        )

pico_add_extra_outputs(conductivity)

