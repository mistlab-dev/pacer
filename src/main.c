/**
 * @file main.c
 * @brief PACER 四旋翼飞控入口 — STM32H743
 *
 * 硬件:
 *   MCU:     STM32H743VI @ 256MHz (APB 64MHz)
 *   IMU:     ICM20948 (I2C1, PB6/PB7)
 *   ESC PWM: TIM1 CH1~4 (PA8, PE11, PE13, PE14)
 *   遥控:    USART3 (PD8/PD9)
 *   调试:    USART1 (PA9/PA10)，鹿小班 CH347 UART0 → COM3，105600 baud
 *
 * 启动流程:
 *   1. HAL_Init() — systick, NVIC
 *   2. SystemClock_Config() — 256MHz, APB 64MHz
 *   3. app_init() — 外设 + 传感器 + 控制器
 *   4. app_run()  — FreeRTOS 调度器 (不返回)
 */

#include "stm32h7xx_hal.h"
#include "app/app.h"
#include "hal/led.h"
#include "usart_printf.h"
#include "app/config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* ================ 系统时钟 256MHz / APB 64MHz ================ */

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
    osc.PLL.PLLM       = 4;   /* HSI 64MHz / 4 = 16MHz */
    osc.PLL.PLLN       = 32;  /* 16MHz * 32 = 512MHz VCO */
    osc.PLL.PLLP       = 2;   /* 512 / 2 = 256MHz SYSCLK */
    osc.PLL.PLLQ       = 10;  /* 512 / 10 = 51.2MHz (USB 近似，换 HSE 后可精调 48MHz) */
    osc.PLL.PLLR       = 2;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { /* 死循环 */ }
    }

    /* SYSCLK 256MHz, HCLK 128MHz, APB1/2/3/4 64MHz */
    clk.ClockType = RCC_CLOCKTYPE_HCLK  |
                    RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1  |
                    RCC_CLOCKTYPE_PCLK2  |
                    RCC_CLOCKTYPE_D3PCLK1 |
                    RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;     /* 256MHz */
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;       /* 128MHz */
    clk.APB1CLKDivider = RCC_APB1_DIV2;     /* 64MHz */
    clk.APB2CLKDivider = RCC_APB2_DIV2;     /* 64MHz */
    clk.APB3CLKDivider = RCC_APB3_DIV2;     /* 64MHz */
    clk.APB4CLKDivider = RCC_APB4_DIV2;     /* 64MHz */

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        while (1) { }
    }

    SystemCoreClockUpdate();
}

/* ================ main ================ */

int main(void)
{
    /* 1. HAL 初始化 */
    HAL_Init();
    SystemClock_Config();

    /* 2. 应用初始化 */
    if (app_init() != 0) {
#if CFG_UART_PLAIN_DEBUG
        uart_puts("PACER MAIN INIT FAIL\r\n");
#else
        printf("[MAIN] init failed!\r\n");
#endif
        while (1) {
            HAL_Delay(500);
            led_tick();
        }
    }

    /* 3. 启动 FreeRTOS (不返回) */
    app_run();

    /* 不应该到这里 */
    while (1) { }
}

/* ================ STM32 HAL 错误回调 ================ */

void Error_Handler(void)
{
#if CFG_UART_PLAIN_DEBUG
    uart_puts("PACER FATAL ERROR\r\n");
#else
    printf("[FATAL] HAL Error\r\n");
#endif
    while (1) { }
}
