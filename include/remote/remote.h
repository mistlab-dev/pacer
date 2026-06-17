/**
 * @file remote.h
 * @brief 遥控接口 — UART 串口
 *
 * STM32H743 版本，替代 RPi 的 UDP/键盘遥控。
 *
 * 使用 USART3 (PD8 TX / PD9 RX), 115200 baud。
 * 协议见 remote.c 注释。
 */

#ifndef PACER_REMOTE_H
#define PACER_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

/* 遥控命令 */
typedef struct {
    float throttle;   /* 0.0~1.0 */
    float roll;       /* -1.0~+1.0 */
    float pitch;      /* -1.0~+1.0 */
    float yaw;        /* -1.0~+1.0 */
    bool  valid;      /* 数据是否有效 */
} remote_cmd_t;

/**
 * @brief 初始化遥控接收
 */
void remote_init(void);
void remote_deinit(void);

/**
 * @brief 读取最新遥控命令
 * @return 0=有新数据, -1=无新数据
 */
int  remote_poll(remote_cmd_t *out);

/**
 * @brief 遥控是否连接 (超时检测)
 */
bool remote_is_connected(void);
void remote_disconnect(void);

/* UART 中断回调 (由 stm32xx_it.c 中的 USART3_IRQHandler 调用) */
void remote_uart_rx_callback(uint8_t b);

#endif /* PACER_REMOTE_H */
