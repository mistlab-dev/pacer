/**
 * @file remote.c
 * @brief 遥控接收 — UART 串口实现
 *
 * 协议 (19 字节帧, v2 — 含 XOR 校验):
 *   [0]     0xAA       帧头 1
 *   [1]     0x55       帧头 2
 *   [2..5]  throttle   float, 0~1,   小端
 *   [6..9]  roll       float, -1~+1, 小端
 *   [10..13] pitch     float, -1~+1, 小端
 *   [14..17] yaw       float, -1~+1, 小端
 *   [18]    XOR        前 18 字节异或校验
 *
 * 解析状态机, USART3 中断接收 → 解析 → 更新 g_cmd。
 */

#include "remote/remote.h"
#include "app/config.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#if CFG_ENABLE_CONSOLE_LOG
#include "usart_printf.h"
#define LOG(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)  ((void)0)
#endif

/* ================ 内部状态 ================ */

static struct {
    remote_cmd_t cmd;
    uint32_t     last_rx_ms;     /* 最后收到数据的毫秒时间 */
    bool         connected;
    bool         new_data;       /* 有新命令 */
    uint32_t     frame_count;    /* 收到的完整帧计数 */
    uint32_t     drop_count;     /* 丢帧/校验失败计数 */
} g;

/* ================ 帧解析 ================ */

#define FRAME_HEADER0   0xAA
#define FRAME_HEADER1   0x55
#define FRAME_SIZE      19        /* 18 字节数据 + 1 字节 XOR 校验 */
#define FRAME_CRC_IDX   (FRAME_SIZE - 1)

static void parse_frame(const uint8_t *frame)
{
    if (frame[0] != FRAME_HEADER0 || frame[1] != FRAME_HEADER1) {
        g.drop_count++;
        return;
    }

    /* XOR 校验: 前 18 字节异或 → 第 19 字节 */
    uint8_t crc = 0;
    for (int i = 0; i < FRAME_SIZE - 1; i++)
        crc ^= frame[i];
    if (crc != frame[FRAME_CRC_IDX]) {
        g.drop_count++;
        return;
    }

    float throttle, roll, pitch, yaw;

    memcpy(&throttle, &frame[2],  4);
    memcpy(&roll,     &frame[6],  4);
    memcpy(&pitch,    &frame[10], 4);
    memcpy(&yaw,      &frame[14], 4);

    /* 钳位 */
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    if (roll < -1.0f) roll = -1.0f;
    if (roll > 1.0f)  roll = 1.0f;
    if (pitch < -1.0f) pitch = -1.0f;
    if (pitch > 1.0f)  pitch = 1.0f;
    if (yaw < -1.0f) yaw = -1.0f;
    if (yaw > 1.0f)  yaw = 1.0f;

    /* 在 ISR 上下文中使用 FromISR 版本 */
    uint32_t saved = taskENTER_CRITICAL_FROM_ISR();
    g.cmd.throttle = throttle;
    g.cmd.roll     = roll;
    g.cmd.pitch    = pitch;
    g.cmd.yaw      = yaw;
    g.cmd.valid    = true;
    g.new_data     = true;
    g.last_rx_ms   = HAL_GetTick();
    g.connected    = true;
    g.frame_count++;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

/* ================ UART 接收 ================ */

/*
 * 方式: USART3 中断接收, 每次收 1 字节, 状态机拼帧。
 * 优点: 简单可靠, 不需要 DMA 配置。
 */

uint8_t  rx_byte;   /* 去掉 static — usart_printf.c ErrorCallback 需 extern */
static uint8_t  rx_buf[FRAME_SIZE];
static uint8_t  rx_idx = 0;

UART_HandleTypeDef huart3;  /* USART3 全局句柄 */

/* ---- USART3 硬件初始化 (UART + NVIC) ----
 * GPIO 配置由 HAL_UART_MspInit() 完成 (PD8 TX / PD9 RX, AF7)
 */
static void remote_uart_init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = CFG_REMOTE_UART_BAUD;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        LOG("[REMOTE] UART3 init FAILED!\r\n");
        return;
    }

    /* 使能 USART3 全局中断 */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    LOG("[REMOTE] UART3 init OK\r\n");
}

static void remote_uart_start(void)
{
    rx_idx = 0;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/* 中断回调: 每收到 1 字节由 HAL 调用 */
void remote_uart_rx_callback(uint8_t b)
{
    switch (rx_idx) {
    case 0:
        if (b == FRAME_HEADER0) {
            rx_buf[rx_idx++] = b;
        }
        break;
    case 1:
        if (b == FRAME_HEADER1) {
            rx_buf[rx_idx++] = b;
        } else {
            rx_idx = 0;  /* 帧头不匹配, 重新找 */
        }
        break;
    default:
        rx_buf[rx_idx++] = b;
        if (rx_idx >= FRAME_SIZE) {
            parse_frame(rx_buf);
            rx_idx = 0;
        }
        break;
    }
}

/* ================ 公开接口 ================ */

void remote_init(void)
{
    memset(&g, 0, sizeof(g));
    g.cmd.throttle = 0.0f;
    g.cmd.roll     = 0.0f;
    g.cmd.pitch    = 0.0f;
    g.cmd.yaw      = 0.0f;
    g.cmd.valid    = false;

    remote_uart_init();
    remote_uart_start();
    LOG("[REMOTE] UART3 ready, baud=%d\r\n", CFG_REMOTE_UART_BAUD);
}

void remote_deinit(void)
{
    HAL_UART_AbortReceive(&huart3);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

int remote_poll(remote_cmd_t *out)
{
    if (!g.new_data) return -1;

    taskENTER_CRITICAL();
    *out = g.cmd;
    g.new_data = false;
    taskEXIT_CRITICAL();

    return 0;
}

bool remote_is_connected(void)
{
    uint32_t elapsed = (HAL_GetTick() - g.last_rx_ms);
    return g.connected && (elapsed < (uint32_t)(CFG_REMOTE_TIMEOUT_SEC * 1000));
}

void remote_disconnect(void)
{
    g.connected = false;
}
