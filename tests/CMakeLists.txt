cmake_minimum_required(VERSION 3.10)
project(pacer_tests LANGUAGES C)

set(CMAKE_C_STANDARD 11)

# 头文件路径
include_directories(
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../include/hal
    ${CMAKE_SOURCE_DIR}/../include/sensor
    ${CMAKE_SOURCE_DIR}/../include/filter
    ${CMAKE_SOURCE_DIR}/../include/ctrl
    ${CMAKE_SOURCE_DIR}/../include/motor
    ${CMAKE_SOURCE_DIR}/../include/app
    ${CMAKE_SOURCE_DIR}/mocks
)

find_library(MATH_LIB m)

# ---- Mock HAL ----
add_library(mock_hal STATIC
    mocks/mock_hal_gpio.c
    mocks/mock_hal_i2c.c
)

# ---- 被测模块 ----
add_library(pacer_modules STATIC
    ${CMAKE_SOURCE_DIR}/../src/ctrl/pid.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/diff_drive.c
    ${CMAKE_SOURCE_DIR}/../src/motor/motor.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(pacer_modules PRIVATE mock_hal ${MATH_LIB})

# ---- 测试可执行文件 ----

# PID 测试 (无 HAL 依赖)
add_executable(test_pid test_pid.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/pid.c
)
target_link_libraries(test_pid PRIVATE ${MATH_LIB})

# Motor 测试 (需要 mock HAL)
add_executable(test_motor test_motor.c
    ${CMAKE_SOURCE_DIR}/../src/motor/motor.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(test_motor PRIVATE mock_hal ${MATH_LIB})

# PCA9685 测试
add_executable(test_pca9685 test_pca9685.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(test_pca9685 PRIVATE mock_hal ${MATH_LIB})

# Diff Drive 测试
add_executable(test_diff_drive test_diff_drive.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/pid.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/diff_drive.c
    ${CMAKE_SOURCE_DIR}/../src/motor/motor.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(test_diff_drive PRIVATE mock_hal ${MATH_LIB})

# ---- 运行所有测试的 target ----
add_custom_target(check
    COMMAND ${CMAKE_BINARY_DIR}/test_pid
    COMMAND ${CMAKE_BINARY_DIR}/test_motor
    COMMAND ${CMAKE_BINARY_DIR}/test_pca9685
    COMMAND ${CMAKE_BINARY_DIR}/test_diff_drive
    DEPENDS test_pid test_motor test_pca9685 test_diff_drive
)
