#!/bin/bash
# 从 GitHub 下载 STM32H7 HAL/CMSIS 关键文件

BASE_URL="https://raw.githubusercontent.com/STMicroelectronics/STM32CubeH7/main"
HAL_DIR="Drivers/STM32H7xx_HAL_Driver"
CMSIS_DIR="Drivers/CMSIS/Device/ST/STM32H7xx"

DEST_STM32HAL="stm32hal"
DEST_CMSIS="cmsis_device"

mkdir -p $DEST_STM32HAL/hal/Inc $DEST_STM32HAL/hal/Src
mkdir -p $DEST_CMSIS/Include $DEST_CMSIS/Source/Templates/gcc

# ===== CMSIS 设备头文件 =====
echo "Downloading CMSIS device headers..."
curl -sL "$BASE_URL/$CMSIS_DIR/Include/stm32h743xx.h" -o $DEST_CMSIS/Include/stm32h743xx.h
curl -sL "$BASE_URL/$CMSIS_DIR/Include/stm32h7xx.h" -o $DEST_CMSIS/Include/stm32h7xx.h
curl -sL "$BASE_URL/$CMSIS_DIR/Include/system_stm32h7xx.h" -o $DEST_CMSIS/Include/system_stm32h7xx.h

# ===== CMSIS 启动文件 =====
echo "Downloading startup file..."
curl -sL "$BASE_URL/$CMSIS_DIR/Source/Templates/gcc/startup_stm32h743xx.s" -o $DEST_CMSIS/Source/Templates/gcc/startup_stm32h743xx.s
curl -sL "$BASE_URL/$CMSIS_DIR/Source/Templates/system_stm32h7xx.c" -o $DEST_CMSIS/Source/Templates/system_stm32h7xx.c

# ===== HAL 库核心头文件 =====
echo "Downloading HAL headers..."
HAL_HEADERS=(
    "stm32h7xx_hal.h"
    "stm32h7xx_hal_conf_template.h"
    "stm32h7xx_hal_cortex.h"
    "stm32h7xx_hal_rcc.h"
    "stm32h7xx_hal_rcc_ex.h"
    "stm32h7xx_hal_gpio.h"
    "stm32h7xx_hal_gpio_ex.h"
    "stm32h7xx_hal_i2c.h"
    "stm32h7xx_hal_i2c_ex.h"
    "stm32h7xx_hal_tim.h"
    "stm32h7xx_hal_tim_ex.h"
    "stm32h7xx_hal_uart.h"
    "stm32h7xx_hal_uart_ex.h"
    "stm32h7xx_hal_pwr.h"
    "stm32h7xx_hal_pwr_ex.h"
    "stm32h7xx_hal_flash.h"
    "stm32h7xx_hal_flash_ex.h"
    "stm32h7xx_hal_dma.h"
    "stm32h7xx_hal_dma_ex.h"
    "stm32h7xx_hal_mdma.h"
    "stm32h7xx_ll_bus.h"
    "stm32h7xx_ll_cortex.h"
    "stm32h7xx_ll_gpio.h"
    "stm32h7xx_ll_i2c.h"
    "stm32h7xx_ll_pwr.h"
    "stm32h7xx_ll_rcc.h"
    "stm32h7xx_ll_system.h"
    "stm32h7xx_ll_tim.h"
    "stm32h7xx_ll_uart.h"
    "stm32h7xx_ll_utils.h"
)

for h in "${HAL_HEADERS[@]}"; do
    curl -sL "$BASE_URL/$HAL_DIR/Inc/$h" -o $DEST_STM32HAL/hal/Inc/$h
done

# ===== HAL 库核心源文件 =====
echo "Downloading HAL sources..."
HAL_SOURCES=(
    "stm32h7xx_hal.c"
    "stm32h7xx_hal_cortex.c"
    "stm32h7xx_hal_rcc.c"
    "stm32h7xx_hal_rcc_ex.c"
    "stm32h7xx_hal_gpio.c"
    "stm32h7xx_hal_i2c.c"
    "stm32h7xx_hal_i2c_ex.c"
    "stm32h7xx_hal_tim.c"
    "stm32h7xx_hal_tim_ex.c"
    "stm32h7xx_hal_uart.c"
    "stm32h7xx_hal_uart_ex.c"
    "stm32h7xx_hal_pwr.c"
    "stm32h7xx_hal_pwr_ex.c"
    "stm32h7xx_hal_flash.c"
    "stm32h7xx_hal_flash_ex.c"
    "stm32h7xx_hal_dma.c"
    "stm32h7xx_hal_dma_ex.c"
    "stm32h7xx_hal_mdma.c"
    "stm32h7xx_ll_bus.c"
    "stm32h7xx_ll_cortex.c"
    "stm32h7xx_ll_gpio.c"
    "stm32h7xx_ll_i2c.c"
    "stm32h7xx_ll_pwr.c"
    "stm32h7xx_ll_rcc.c"
    "stm32h7xx_ll_system.c"
    "stm32h7xx_ll_tim.c"
    "stm32h7xx_ll_uart.c"
    "stm32h7xx_ll_utils.c"
)

for s in "${HAL_SOURCES[@]}"; do
    curl -sL "$BASE_URL/$HAL_DIR/Src/$s" -o $DEST_STM32HAL/hal/Src/$s
done

echo "Done. Files downloaded to stm32hal/ and cmsis_device/"