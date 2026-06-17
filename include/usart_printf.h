/**
 * @file usart_printf.h
 * @brief UART printf 重定向 — 通过 USART2 输出调试日志
 *
 * 使用方法:
 *   1. 在 main 中调用 usart_printf_init()
 *   2. 正常使用 printf() 即可输出到 UART
 *
 * 硬件: USART2 PA2(TX) / PA3(RX), 115200 baud
 */

#ifndef USART_PRINTF_H
#define USART_PRINTF_H

#include <stdio.h>

/**
 * @brief 初始化调试 UART
 * 配置 USART2, 重定向 stdout
 */
void usart_printf_init(void);

#endif /* USART_PRINTF_H */
