/**
 * @file stm32h7xx_it.c
 * @brief STM32 中断处理 — USART3 (遥控) + SysTick
 */

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

extern UART_HandleTypeDef huart3;
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);

/* USART3 全局中断 */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* FreeRTOS SysTick */
void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* HardFault 等异常处理见 src/main.c */
