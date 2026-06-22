/**
 * @file usart_printf.c
 * @brief UART printf 重定向 + MSP 实现 — STM32H743
 */

#include "usart_printf.h"
#include "app/config.h"
#include "app/cli.h"
#include "remote/remote.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>

UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static SemaphoreHandle_t uart_tx_mtx;

#if (CFG_DEBUG_UART_PORT == PACER_DEBUG_USART1)
#  define DBG_UART_INSTANCE     USART1
#  define DBG_UART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#  define DBG_UART_CLK_DISABLE() __HAL_RCC_USART1_CLK_DISABLE()
#  define DBG_UART_AF           GPIO_AF7_USART1
#  define DBG_UART_TX_PIN       GPIO_PIN_9
#  define DBG_UART_RX_PIN       GPIO_PIN_10
#elif (CFG_DEBUG_UART_PORT == PACER_DEBUG_USART2)
#  define DBG_UART_INSTANCE     USART2
#  define DBG_UART_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE()
#  define DBG_UART_CLK_DISABLE() __HAL_RCC_USART2_CLK_DISABLE()
#  define DBG_UART_AF           GPIO_AF7_USART2
#  define DBG_UART_TX_PIN       GPIO_PIN_2
#  define DBG_UART_RX_PIN       GPIO_PIN_3
#else
#  error "CFG_DEBUG_UART_PORT must be PACER_DEBUG_USART1 or PACER_DEBUG_USART2"
#endif

void usart_printf_init(void)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USART16;
    pclk.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    huart2.Instance        = DBG_UART_INSTANCE;
    huart2.Init.BaudRate   = CFG_DEBUG_UART_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

#if CFG_UART_PLAIN_DEBUG
    /* 空板联调：不用 FreeRTOS 互斥，避免调度器未启动时 API 副作用 */
    uart_tx_mtx = NULL;
#else
    uart_tx_mtx = xSemaphoreCreateMutex();
#endif
    setvbuf(stdout, NULL, _IONBF, 0);

    static const char boot[] = "PACER BOOT\r\n";
    uart_write_bytes((const uint8_t *)boot, (uint16_t)(sizeof(boot) - 1));
}

void uart_write_bytes(const uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0) {
        return;
    }

    if (uart_tx_mtx != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreTake(uart_tx_mtx, portMAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);

    if (uart_tx_mtx != NULL &&
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGive(uart_tx_mtx);
    }
}

void uart_puts(const char *s)
{
    if (!s) {
        return;
    }

    uint16_t len = 0;
    while (s[len] != '\0') {
        len++;
    }
    uart_write_bytes((const uint8_t *)s, len);
}

void usart_debug_nvic_enable(void)
{
#if (CFG_DEBUG_UART_PORT == PACER_DEBUG_USART1)
    HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#else
    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DBG_UART_INSTANCE) {
        DBG_UART_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef g = {0};
        g.Pin       = DBG_UART_TX_PIN | DBG_UART_RX_PIN;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = DBG_UART_AF;
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


void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DBG_UART_INSTANCE) {
        HAL_GPIO_DeInit(GPIOA, DBG_UART_TX_PIN | DBG_UART_RX_PIN);
        DBG_UART_CLK_DISABLE();
    }

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
        __HAL_RCC_USART3_CLK_DISABLE();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    } else if (huart->Instance == DBG_UART_INSTANCE) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(huart, &cli_rx_byte, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DBG_UART_INSTANCE) {
        cli_rx_byte_cb(cli_rx_byte);
        HAL_UART_Receive_IT(&huart2, &cli_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        remote_uart_rx_callback(rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
