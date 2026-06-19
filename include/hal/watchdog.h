/**
 * @file watchdog.h
 * @brief 独立看门狗 (IWDG) 模块
 *
 * 如果 ctrl_task 卡死（IMU I2C 死锁、硬 fault 等），
 * 看门狗溢出后会复位 MCU，防止电机失控。
 *
 * 窗口看门狗 (WWDG) 保留给未来使用，当前用 IWDG。
 */

#ifndef PACER_WATCHDOG_H
#define PACER_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化独立看门狗
 * @param timeout_ms 超时时间 (ms)，建议 500ms
 */
void watchdog_init(uint32_t timeout_ms);

/**
 * @brief 喂狗 (在 ctrl_task 主循环中调用)
 */
void watchdog_kick(void);

#endif /* PACER_WATCHDOG_H */
