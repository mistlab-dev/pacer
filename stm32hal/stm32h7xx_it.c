/**
 * @file stm32h7xx_it.c
 * @brief STM32 中断处理 — USART3 (遥控) + SysTick + 异常处理
 *
 * SVC_Handler / PendSV_Handler 由 FreeRTOS port.c 提供，
 * startup 汇编向量表中的弱符号会被 port.c 的实现覆盖。
 * 不要在此文件中重新定义 SVC/PendSV。
 */

#include "stm32h7xx_hal.h"
#include "app/config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

extern void xPortSysTickHandler(void);

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern void remote_uart_rx_callback(uint8_t b);

/* ============ 系统异常处理 ============ */

/* HardFault — 提取栈帧用于调试，然后等看门狗复位 */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "tst   lr, #4               \n"
        "ite   eq                    \n"
        "mrseq r0, msp               \n"
        "mrsne r0, psp               \n"
        "ldr   r1, =hard_fault_dump  \n"
        "bx    r1                    \n"
    );
}

void hard_fault_dump(uint32_t *sf)
{
    /* sf[0..7] = R0, R1, R2, R3, R12, LR, PC, PSR */
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
        "\r\n*** HARD FAULT ***\r\n"
        "PC=%08lX LR=%08lX PSR=%08lX\r\n"
        "R0=%08lX R1=%08lX R2=%08lX R3=%08lX R12=%08lX\r\n",
        (unsigned long)sf[6], (unsigned long)sf[5], (unsigned long)sf[7],
        (unsigned long)sf[0], (unsigned long)sf[1], (unsigned long)sf[2],
        (unsigned long)sf[3], (unsigned long)sf[4]);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 200);

    /* 停电机 (通过直接写 PWM 寄存器, 不依赖子系统) */
    extern TIM_HandleTypeDef htim1;
    if (htim1.Instance) {
        htim1.Instance->CCR1 = 1000;
        htim1.Instance->CCR2 = 1000;
        htim1.Instance->CCR3 = 1000;
        htim1.Instance->CCR4 = 1000;
    }

    for (;;);  /* 等看门狗复位 */
}

void MemManage_Handler(void)
{
    const char *msg = "\r\n*** MEM MANAGE FAULT ***\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    for (;;);
}

void BusFault_Handler(void)
{
    const char *msg = "\r\n*** BUS FAULT ***\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    for (;;);
}

void UsageFault_Handler(void)
{
    const char *msg = "\r\n*** USAGE FAULT ***\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    for (;;);
}

/* ============ FreeRTOS 钩子函数 ============ */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    char buf[80];
    int n = snprintf(buf, sizeof(buf),
        "\r\n[FATAL] stack overflow: %s\r\n", pcTaskName);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, 100);
    for (;;);
}

void vApplicationMallocFailedHook(void)
{
    const char *msg = "\r\n[FATAL] malloc failed\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
    for (;;);
}

/* ============ 调试 UART 全局中断 (CLI) ============ */

void USART1_IRQHandler(void)
{
#if (CFG_DEBUG_UART_PORT == PACER_DEBUG_USART1)
    HAL_UART_IRQHandler(&huart2);
#endif
}

void USART2_IRQHandler(void)
{
#if (CFG_DEBUG_UART_PORT == PACER_DEBUG_USART2)
    HAL_UART_IRQHandler(&huart2);
#endif
}

/* ============ USART3 全局中断 (遥控接收) ============ */

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* ============ SysTick (FreeRTOS tick) ============ */

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
