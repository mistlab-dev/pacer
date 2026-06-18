/**
 * @file main.c
 * @brief PACER 四旋翼飞控入口 — STM32H743
 *
 * 硬件:
 *   MCU:     STM32H743VI @ 480MHz
 *   IMU:     ICM20948 (I2C1, PB6/PB7)
 *   ESC PWM: TIM1 CH1~4 (PA8, PE11, PE13, PE14)
 *   遥控:    USART3 (PD8/PD9)
 *   调试:    USART2 (PA2/PA3)
 *
 * 启动流程:
 *   1. HAL_Init() — systick, NVIC
 *   2. SystemClock_Config() — 480MHz
 *   3. app_init() — 外设 + 传感器 + 控制器
 *   4. app_run()  — FreeRTOS 调度器 (不返回)
 */

#include "stm32h7xx_hal.h"
#include "app/app.h"
#include "usart_printf.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* ================ 系统时钟 480MHz ================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* LSI — 看门狗用 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.LSIState       = RCC_LSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = 4;   /* HSI=64MHz / 4 = 16MHz */
    osc.PLL.PLLN       = 60;  /* 16MHz * 60 = 960MHz VCO (H743 max) */
    osc.PLL.PLLP       = 2;   /* 960 / 2 = 480MHz SYS ✓ */
    osc.PLL.PLLQ       = 15;  /* USB 48MHz */
    osc.PLL.PLLR       = 2;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { /* 死循环 */ }
    }

    /* CPU 480MHz, AXI 240MHz, APB1 120MHz, APB2 240MHz */
    clk.ClockType = RCC_CLOCKTYPE_HCLK  |
                    RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1  |
                    RCC_CLOCKTYPE_PCLK2  |
                    RCC_CLOCKTYPE_D3PCLK1 |
                    RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;     /* 480MHz */
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;       /* 240MHz AXI */
    clk.APB1CLKDivider = RCC_APB1_DIV2;       /* 120MHz */
    clk.APB2CLKDivider = RCC_APB2_DIV2;       /* 240MHz */
    clk.APB3CLKDivider = RCC_APB3_DIV2;       /* 120MHz */
    clk.APB4CLKDivider = RCC_APB4_DIV2;       /* 120MHz */

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        while (1) { }
    }
}

/* ================ main ================ */

int main(void)
{
    /* 1. HAL 初始化 */
    HAL_Init();
    SystemClock_Config();

    /* 2. 应用初始化 */
    if (app_init() != 0) {
        printf("[MAIN] init failed!\r\n");
        while (1) { HAL_Delay(1000); }
    }

    /* 3. 启动 FreeRTOS (不返回) */
    app_run();

    /* 不应该到这里 */
    while (1) { }
}

/* ================ STM32 HAL 错误回调 ================ */

void Error_Handler(void)
{
    printf("[FATAL] HAL Error\r\n");
    while (1) { }
}
