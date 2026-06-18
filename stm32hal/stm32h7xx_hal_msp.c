/**
 * @file stm32h7xx_hal_msp.c
 * @brief STM32 HAL MSP — 底层外设初始化
 *
 * HAL_UART_MspInit / HAL_UART_MspDeInit 由 usart_printf.c 提供
 * (已覆盖 USART2 调试 + USART3 遥控)。
 *
 * 本文件提供 I2C1、TIM1、ADC1 的 MSP 初始化。
 */

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

/**
 * @brief I2C1 MSP 初始化 — PB6(SCL) / PB7(SDA), 400kHz
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitTypeDef g = {0};
        g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        g.Mode      = GPIO_MODE_AF_OD;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

/**
 * @brief I2C1 MSP 反初始化
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        __HAL_RCC_I2C1_CLK_DISABLE();
    }
}

/**
 * @brief TIM1 MSP 初始化 — 由 hal_tim.c 的 tim1_gpio_init 已处理
 *        这里提供空实现防止 HAL 内部 callback 崩溃
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

/**
 * @brief ADC1 MSP 初始化 — PA5 模拟输入
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef g = {0};
        g.Pin  = GPIO_PIN_5;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &g);
    }
}

/**
 * @brief ADC1 MSP 反初始化
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
        __HAL_RCC_ADC1_CLK_DISABLE();
    }
}
