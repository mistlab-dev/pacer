/**
 * @file stm32h7xx_hal_msp.c
 * @brief STM32 HAL MSP — 底层外设初始化 (占位)
 *
 * 实际项目通常由 CubeMX 生成。这里提供必要的空实现，
 * GPIO/Clock 配置已分散到各 hal_*.c 中。
 */

#include "stm32h7xx_hal.h"

/* HAL 时基 — 使用 SysTick (FreeRTOS 会接管) */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);
    return HAL_OK;
}

void HAL_Delay(uint32_t Delay)
{
    /* 简单循环延时 — 在调度器启动前使用 */
    uint32_t cycles = Delay * (SystemCoreClock / 1000U / 8U);
    for (uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}

uint32_t HAL_GetTick(void)
{
    /* FreeRTOS 接管后使用 xTaskGetTickCount()
     * 启动前用 DWT 计数器 */
    static uint32_t counter = 0;
    return counter++;
}
