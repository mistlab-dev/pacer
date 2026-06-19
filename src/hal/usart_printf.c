/**
 * @file usart_printf.c
 * @brief UART printf 重定向 + MSP 实现 — STM32H743
 *
 * PA2(TX) / PA3(RX), 115200 8N1
 * 重定向 fputc → HAL_UART_Transmit
 *
 * 本文件提供 HAL_UART_MspInit / HAL_UART_MspDeInit
 * 覆盖 USART2 (调试) 和 USART3 (遥控)。
 */

#include "usart_printf.h"
#include "app/cli.h"
#include "remote/remote.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>

UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void usart_printf_init(void)
{
    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* 关闭 stdout 缓冲, printf 立即输出 */
    setvbuf(stdout, NULL, _IONBF, 0);
}

/* HAL_UART_MspInit — 底层 GPIO/Clock 配置 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef g = {0};
        g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);
    }

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        GPIO_InitTypeDef g = {0};
        g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &g);
    }
}

/* printf 重定向 */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t b = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &b, 1, 10);
    return ch;
}

/* USART MSP DeInit */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        __HAL_RCC_USART2_CLK_DISABLE();
    }

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
        __HAL_RCC_USART3_CLK_DISABLE();
    }
}

/* UART 错误回调 — 防止 Overrun/Noise/Framing 错误导致接收永久死锁 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    } else if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(huart, &cli_rx_byte, 1);
    }
}

/* USART2=CLI, USART3=遥控 — 统一分发 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        cli_rx_byte_cb(cli_rx_byte);
        HAL_UART_Receive_IT(&huart2, &cli_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        remote_uart_rx_callback(rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
