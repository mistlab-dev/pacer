# CMakeLists.txt — PACER 四旋翼飞控 (STM32H743)
#
# 交叉编译: ARM GCC (arm-none-eabi-)
# Windows: scripts\build.bat
# Linux:   scripts/build.sh

cmake_minimum_required(VERSION 3.16)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 在 project() 之前设置编译器参数
set(CMAKE_C_FLAGS_INIT "-B/opt/arm-none-eabi-bin -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
set(CMAKE_ASM_FLAGS_INIT "-B/opt/arm-none-eabi-bin -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")

project(pacer C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(
    -B/opt/arm-none-eabi-bin
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Os
    -g
)

add_link_options(
    -B/opt/arm-none-eabi/bin
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/cmake/stm32h743vi_flash.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/pacer.map
)

include_directories(
    include
    cmsis/Include
    stm32hal/cmsis_device/Include
    stm32hal/hal/Inc
    freertos
    freertos/include
    freertos/portable/GCC/ARM_CM7/r0p1
)

set(PACER_SOURCES
    cmake/startup_stm32h743xx.s

    src/main.c
    src/hal/usart_printf.c
    src/hal/system_stm32h7xx.c
    src/hal/syscalls.c
    stm32hal/stm32h7xx_it.c
    stm32hal/stm32h7xx_hal_msp.c

    src/hal/hal_i2c.c
    src/hal/hal_gpio.c
    src/hal/hal_tim.c
    src/hal/led.c
    src/hal/watchdog.c
    src/hal/battery.c

    stm32hal/hal/Src/stm32h7xx_hal.c
    stm32hal/hal/Src/stm32h7xx_hal_cortex.c
    stm32hal/hal/Src/stm32h7xx_hal_rcc.c
    stm32hal/hal/Src/stm32h7xx_hal_rcc_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_gpio.c
    stm32hal/hal/Src/stm32h7xx_hal_i2c.c
    stm32hal/hal/Src/stm32h7xx_hal_i2c_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_tim.c
    stm32hal/hal/Src/stm32h7xx_hal_tim_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_uart.c
    stm32hal/hal/Src/stm32h7xx_hal_uart_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_pwr.c
    stm32hal/hal/Src/stm32h7xx_hal_pwr_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_flash.c
    stm32hal/hal/Src/stm32h7xx_hal_flash_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_dma.c
    stm32hal/hal/Src/stm32h7xx_hal_dma_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_exti.c
    stm32hal/hal/Src/stm32h7xx_hal_adc.c
    stm32hal/hal/Src/stm32h7xx_hal_adc_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_iwdg.c

    freertos/tasks.c
    freertos/queue.c
    freertos/list.c
    freertos/timers.c
    freertos/portable/GCC/ARM_CM7/r0p1/port.c
    freertos/portable/MemMang/heap_4.c

    src/sensor/imu.c
    src/sensor/impl/imu_icm20948.c
    src/filter/filter.c
    src/ctrl/pid.c
    src/ctrl/attitude.c
    src/ctrl/quad_mixer.c
    src/motor/motor.c
    src/remote/remote.c
    src/app/app.c
    src/app/cli.c
)

add_compile_definitions(
    TARGET_STM32H743=1
    USE_HAL_DRIVER
    STM32H743xx
    USE_PWR_LDO_SUPPLY
)

add_executable(pacer.elf ${PACER_SOURCES})

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
