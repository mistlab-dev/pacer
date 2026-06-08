cmake_minimum_required(VERSION 3.10)
project(pacer VERSION 1.0.0 LANGUAGES C)

# C11 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror=return-type")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG   "-g -DDEBUG -O0")

# 依赖
find_package(Threads REQUIRED)
find_library(PIGPIO_LIB pigpio)
find_library(MATH_LIB m)

# OpenCV (可选 — 用于摄像头追踪)
option(USE_OPENCV "Enable OpenCV camera tracking" OFF)
if(USE_OPENCV)
    find_package(OpenCV REQUIRED)
    add_definitions(-DUSE_OPENCV)
endif()

# 源文件 (按模块组织)
set(SOURCES
    # HAL
    src/hal/hal_i2c.c
    src/hal/hal_gpio.c

    # 传感器
    src/sensor/imu.c
    src/sensor/impl/imu_icm20948.c

    # 滤波器
    src/filter/filter.c

    # 控制器
    src/ctrl/pid.c
    src/ctrl/balance.c

    # 电机
    src/motor/motor.c

    # 追踪
    src/tracker/beacon.c
    src/tracker/camera.c
    src/tracker/tracker.c

    # 应用
    src/app/app.c
    src/main.c
)

# 头文件搜索路径
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/hal
    ${CMAKE_SOURCE_DIR}/include/sensor
    ${CMAKE_SOURCE_DIR}/include/filter
    ${CMAKE_SOURCE_DIR}/include/ctrl
    ${CMAKE_SOURCE_DIR}/include/motor
    ${CMAKE_SOURCE_DIR}/include/tracker
    ${CMAKE_SOURCE_DIR}/include/app
)

# 构建目标
add_executable(pacer ${SOURCES})

target_link_libraries(pacer
    PRIVATE
    Threads::Threads
    ${PIGPIO_LIB}
    ${MATH_LIB}
    rt
)

if(USE_OPENCV)
    target_link_libraries(pacer PRIVATE ${OpenCV_LIBS})
endif()

# 安装
install(TARGETS pacer RUNTIME DESTINATION bin)
