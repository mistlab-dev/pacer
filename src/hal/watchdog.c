/**
 * @file watchdog.c
 * @brief STM32H743 独立看门狗 (IWDG) 实现
 *
 * LSI 时钟 ≈ 32kHz
 * 预分频 64 → 500Hz (2ms/tick)
 * 重载值 = timeout_ms / 2
 *
 * 例: 500ms → reload = 250
 */

#include "hal/watchdog.h"
#include "stm32h7xx_hal.h"

static IWDG_HandleTypeDef hiwdg;

void watchdog_init(uint32_t timeout_ms)
{
    hiwdg.Instance       = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;   /* 32kHz / 64 = 500Hz */
    hiwdg.Init.Reload    = (timeout_ms * 500) / 1000;
    if (hiwdg.Init.Reload == 0) hiwdg.Init.Reload = 1;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        /* 看门狗初始化失败是严重问题，但这里只能记录 */
    }
}

void watchdog_kick(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
