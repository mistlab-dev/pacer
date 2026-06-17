/**
 * @file stm32h7xx_it.c
 * @brief STM32 中断处理 — USART3 (遥控) + SysTick
 */

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"

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
    /* FreeRTOS 在 port.c 中处理 */
}

/* HardFault */
void HardFault_Handler(void)   { while (1) {} }
void MemManage_Handler(void)   { while (1) {} }
void BusFault_Handler(void)    { while (1) {} }
void UsageFault_Handler(void)  { while (1) {} }
void NMI_Handler(void)         { while (1) {} }
