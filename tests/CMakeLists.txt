cmake_minimum_required(VERSION 3.10)
project(pacer_tests LANGUAGES C)

set(CMAKE_C_STANDARD 11)

include_directories(
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/mocks
)

find_library(MATH_LIB m)

# ---- Mock HAL ----
add_library(mock_hal STATIC
    mocks/mock_hal_gpio.c
    mocks/mock_hal_i2c.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)

# ---- 测试可执行文件 ----

# PID 单元测试
add_executable(test_pid test_pid.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/pid.c
)
target_link_libraries(test_pid PRIVATE ${MATH_LIB})

# Motor 单元测试 (需要 mock HAL)
add_executable(test_motor test_motor.c
    ${CMAKE_SOURCE_DIR}/../src/motor/motor.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(test_motor PRIVATE mock_hal ${MATH_LIB})

# PCA9685 单元测试
add_executable(test_pca9685 test_pca9685.c
    ${CMAKE_SOURCE_DIR}/../src/hal/hal_pca9685.c
)
target_link_libraries(test_pca9685 PRIVATE mock_hal ${MATH_LIB})

# 四旋翼控制逻辑模拟 (PID + 姿态 + 混控 + 物理模型 + 自检 + 飞行阶段)
add_executable(sim_quad sim_quad.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/pid.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/attitude.c
    ${CMAKE_SOURCE_DIR}/../src/ctrl/quad_mixer.c
    ${CMAKE_SOURCE_DIR}/../src/app/preflight.c
    ${CMAKE_SOURCE_DIR}/../src/app/flight_phase.c
)
target_link_libraries(sim_quad PRIVATE ${MATH_LIB})

# ---- 运行所有测试 ----
add_custom_target(check
    COMMAND ${CMAKE_BINARY_DIR}/test_pid
    COMMAND ${CMAKE_BINARY_DIR}/test_motor
    COMMAND ${CMAKE_BINARY_DIR}/test_pca9685
    COMMAND ${CMAKE_BINARY_DIR}/sim_quad
    DEPENDS test_pid test_motor test_pca9685 sim_quad
)
