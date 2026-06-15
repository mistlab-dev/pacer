cmake_minimum_required(VERSION 3.10)
project(pacer C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 目标板选择
set(TARGET_BOARD "rpi_zero2w" CACHE STRING "Target board: rpi_zero (ARMv6) or rpi_zero2w (ARMv8)")
set_property(CACHE TARGET_BOARD PROPERTY STRINGS "rpi_zero" "rpi_zero2w")

if(TARGET_BOARD STREQUAL "rpi_zero")
    # 原版 Zero: 单核 ARMv6 (BCM2835) — 交叉编译或板上编译
    add_compile_definitions(TARGET_RPI_ZERO=1 BOARD_SINGLE_CORE=1)
    # ARM march 选项仅在交叉编译工具链下生效
    if(CMAKE_C_COMPILER_TARGET OR CMAKE_CROSSCOMPILING)
        add_compile_options(-march=armv6 -mfpu=vfp -mfloat-abi=hard)
    endif()
    message(STATUS "Target: Raspberry Pi Zero (ARMv6, single-core, 200Hz control)")
elseif(TARGET_BOARD STREQUAL "rpi_zero2w")
    # Zero 2W: 四核 ARMv8 (BCM2837A0)
    add_compile_definitions(TARGET_RPI_ZERO2W=1 BOARD_QUAD_CORE=1)
    if(CMAKE_CROSSCOMPILING)
        add_compile_options(-march=armv8-a -mtune=cortex-a53)
    endif()
    message(STATUS "Target: Raspberry Pi Zero 2W (ARMv8, quad-core, 400Hz control)")
else()
    message(FATAL_ERROR "Unknown TARGET_BOARD: ${TARGET_BOARD}")
endif()

# 警告
add_compile_options(-Wall -Wextra -Wshadow -Wundef)

# pigpio (目标机器上有, 开发机可以没有)
find_library(PIGPIO pigpio QUIET)
find_library(PIGPIO_RT rt QUIET)
if(PIGPIO)
    message(STATUS "pigpio found: ${PIGPIO}")
else()
    message(WARNING "pigpio not found — stub for cross-compile")
endif()

# 源文件
set(SRCS
    src/main.c
    src/app/app.c
    src/hal/hal_gpio.c
    src/hal/hal_i2c.c
    src/hal/hal_pca9685.c
    src/sensor/imu.c
    src/sensor/impl/imu_icm20948.c
    src/filter/filter.c
    src/ctrl/pid.c
    src/ctrl/attitude.c
    src/ctrl/quad_mixer.c
    src/motor/motor.c
    src/remote/remote.c
)

add_executable(${PROJECT_NAME} ${SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    m
)

# pigpio 和 rt (只在找到时链接)
if(PIGPIO)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PIGPIO})
endif()
if(PIGPIO_RT)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${PIGPIO_RT})
endif()

# ---------- 测试 (可选) ----------
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
    enable_testing()

    add_executable(test_pid
        tests/test_pid.c
        src/ctrl/pid.c
    )
    target_include_directories(test_pid PRIVATE ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME pid COMMAND test_pid)

    add_executable(test_motor
        tests/test_motor.c
        src/motor/motor.c
        tests/mocks/mock_hal_gpio.c
        tests/mocks/mock_hal_i2c.c
    )
    target_include_directories(test_motor PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/tests/mocks
    )
    add_test(NAME motor COMMAND test_motor)

    add_executable(test_pca9685
        tests/test_pca9685.c
        src/hal/hal_pca9685.c
        tests/mocks/mock_hal_i2c.c
    )
    target_include_directories(test_pca9685 PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/tests/mocks
    )
    add_test(NAME pca9685 COMMAND test_pca9685)
endif()
