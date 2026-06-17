# CMakeLists.txt — PACER 四旋翼飞控 (STM32H743)
#
# 交叉编译: ARM GCC (arm-none-eabi-)
# 工具链文件: cmake/toolchain-arm-none-eabi.cmake
#
# 构建:
#   mkdir build && cd build
#   cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain-arm-none-eabi.cmake ..
#   make -j$(nproc)

cmake_minimum_required(VERSION 3.16)

project(pacer C)

# ---- 编译选项 ----
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Os
    -g
)

add_link_options(
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/cmake/stm32h743vi_flash.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/pacer.map
    -nostartfiles
)

# ---- 头文件搜索路径 ----
include_directories(
    include
    cmsis           # STM32H7 CMSIS 头文件 (需补充)
    stm32hal        # STM32 HAL 头文件 (需补充)
    freertos/Source/include
    freertos/Source/portable/GCC/ARM_CM7/r0p1
)

# ---- 源文件 ----
set(PACER_SOURCES
    # 启动文件 (需补充: 从 CubeMX 生成)
    # cmake/startup_stm32h743xx.s

    # 入口
    src/main.c
    src/hal/usart_printf.c

    # HAL 层
    src/hal/hal_i2c.c
    src/hal/hal_gpio.c
    src/hal/hal_tim.c

    # 传感器
    src/sensor/imu.c
    src/sensor/impl/imu_icm20948.c

    # 滤波
    src/filter/filter.c

    # 控制
    src/ctrl/pid.c
    src/ctrl/attitude.c
    src/ctrl/quad_mixer.c

    # 电机
    src/motor/motor.c

    # 遥控
    src/remote/remote.c

    # 应用
    src/app/app.c

    # STM32 系统占位 (需补充: 从 CubeMX 生成)
    # stm32hal/stm32h7xx_hal.c
    # stm32hal/stm32h7xx_hal_msp.c
    # stm32hal/stm32h7xx_it.c

    # FreeRTOS (需补充: 从 FreeRTOS 仓库获取)
    # freertos/Source/tasks.c
    # freertos/Source/queue.c
    # freertos/Source/list.c
    # freertos/Source/portable/GCC/ARM_CM7/r0p1/port.c
    # freertos/Source/portable/MemMang/heap_4.c
)

# ---- 编译定义 ----
add_compile_definitions(
    TARGET_STM32H743=1
    USE_HAL_DRIVER
    STM32H743xx
    ARM_MATH_CM7
)

# ---- 可执行文件 ----
add_executable(pacer.elf ${PACER_SOURCES})

# ---- 生成 bin/hex ----
find_program(ARM_SIZE     arm-none-eabi-size)
find_program(ARM_OBJCOPY  arm-none-eabi-objcopy)

if(ARM_SIZE)
    add_custom_command(TARGET pacer.elf POST_BUILD
        COMMAND ${ARM_SIZE} $<TARGET_FILE:pacer.elf>
        COMMENT "Size report"
    )
endif()

if(ARM_OBJCOPY)
    add_custom_command(TARGET pacer.elf POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary $<TARGET_FILE:pacer.elf> ${CMAKE_BINARY_DIR}/pacer.bin
        COMMENT "Generating pacer.bin"
    )
    add_custom_command(TARGET pacer.elf POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O ihex $<TARGET_FILE:pacer.elf> ${CMAKE_BINARY_DIR}/pacer.hex
        COMMENT "Generating pacer.hex"
    )
endif()
