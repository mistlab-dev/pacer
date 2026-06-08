cmake_minimum_required(VERSION 3.10)
project(pacer VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler flags for Raspberry Pi
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")

# Find threads
find_package(Threads REQUIRED)

# pigpio for GPIO/PWM/I2C
find_library(PIGPIO_LIB pigpio)
find_library(PIGPIO_RT pigpiod_if2)

# Math library
find_library(MATH_LIB m)

# Source files
file(GLOB_RECURSE SOURCES "src/*.c")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/drivers
    ${CMAKE_SOURCE_DIR}/include/control
    ${CMAKE_SOURCE_DIR}/include/sensors
    ${CMAKE_SOURCE_DIR}/include/utils
)

add_executable(pacer ${SOURCES})

target_link_libraries(pacer
    Threads::Threads
    ${PIGPIO_LIB}
    ${PIGPIO_RT}
    ${MATH_LIB}
    rt
)

# Install
install(TARGETS pacer DESTINATION bin)
install(DIRECTORY config/ DESTINATION /etc/pacer)
