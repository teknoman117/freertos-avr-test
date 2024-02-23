cmake_minimum_required(VERSION 3.20)
project(freertos-test VERSION 0.1)

set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)

set(C_STANDARD 11)
set(C_EXTENSIONS OFF)

set(F_CPU 16000000UL)
set(MCU atmega328p)
set(BAUD 115200)

add_definitions(-DF_CPU=${F_CPU})
add_compile_options(-mmcu=${MCU})
add_link_options(-mmcu=${MCU})

add_subdirectory(dependencies)
add_subdirectory(src)
