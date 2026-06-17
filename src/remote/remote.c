/**
 * @file remote.c
 * @brief 遥控接收 — UART 串口实现
 *
 * 替代之前的 UDP + 键盘方式。
 *
 * 协议 (18 字节帧):
 *   [0]     0xAA       帧头 1
 *   [1]     0x55       帧头 2
 *   [2..5]  throttle   float, 0~1,   小端
 *   [6..9]  roll       float, -1~+1, 小端
 *   [10..13] pitch     float, -1~+1, 小端
 *   [14..17] yaw       float, -1~+1, 小端
 *
 * 解析状态机, 中断/DMA 接收 → 解析 → 更新 g_cmd。
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
} g;

/* ================ 帧解析 ================ */

#define FRAME_HEADER0   0xAA
#define FRAME_HEADER1   0x55
#define FRAME_SIZE      18

static void parse_frame(const uint8_t *frame)
{
    if (frame[0] != FRAME_HEADER0 || frame[1] != FRAME_HEADER1)
        return;

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

    /* 原子更新 (在 FreeRTOS 下建议用 taskENTER_CRITICAL) */
    taskENTER_CRITICAL();
    g.cmd.throttle = throttle;
    g.cmd.roll     = roll;
    g.cmd.pitch    = pitch;
    g.cmd.yaw      = yaw;
    g.cmd.valid    = true;
    g.new_data     = true;
    g.last_rx_ms   = HAL_GetTick();
    g.connected    = true;
    taskEXIT_CRITICAL();
}

/* ================ UART 接收 ================ */

/*
 * 方式: USART3 中断接收, 每次收 1 字节, 状态机拼帧。
 * 优点: 简单可靠, 不需要 DMA 配置。
 */

static uint8_t  rx_byte;
static uint8_t  rx_buf[FRAME_SIZE];
static uint8_t  rx_idx = 0;
static bool     rx_busy = false;

UART_HandleTypeDef huart3;  /* USART3 全局句柄 */

void remote_uart_start(void)
{
    rx_idx = 0;
    rx_busy = true;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

/* USART3 RX 中断回调 (由 HAL_UART_RxCpltCallback 调用) */
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
            /* 完整帧 */
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

    /* 启动中断接收 */
    remote_uart_start();
    LOG("[REMOTE] UART3 ready, baud=%d\r\n", CFG_REMOTE_UART_BAUD);
}

void remote_deinit(void)
{
    rx_busy = false;
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
