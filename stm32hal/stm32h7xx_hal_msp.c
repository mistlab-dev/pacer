/**
 * @file stm32h7xx_hal_msp.c
 * @brief STM32 HAL MSP — 底层外设初始化
 *
 * uwTick / HAL_GetTick / HAL_IncTick / HAL_Delay 均由 HAL 库提供。
 * SysTick 由 stm32h7xx_it.c 的 SysTick_Handler 驱动 (调用 HAL_IncTick)。
 *
 * 本文件提供 HAL_UART_MspInit — 配置 USART3 GPIO。
 */

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart3;

/**
 * @brief UART MSP 初始化 — 配置 USART3 GPIO (PD8 TX / PD9 RX)
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_InitTypeDef gpio = {0};
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Alternate = GPIO_AF7_USART3;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;

        /* PD8 = USART3_TX */
        gpio.Pin = GPIO_PIN_8;
        HAL_GPIO_Init(GPIOD, &gpio);

        /* PD9 = USART3_RX */
        gpio.Pin = GPIO_PIN_9;
        HAL_GPIO_Init(GPIOD, &gpio);
    }
}

/**
 * @brief UART MSP 反初始化
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
        __HAL_RCC_USART3_CLK_DISABLE();
    }
}
