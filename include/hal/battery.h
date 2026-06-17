/**
 * @file battery.h
 * @brief 电池电压检测模块
 *
 * 通过 ADC 读取分压电路监测电池电压。
 * 低压警告 → LED 快闪；临界低压 → 自动降落/停机。
 *
 * 硬件:
 *   ADC1 CH5 (PA5) ← 分压网络
 *   电池: 3S LiPo (11.1V ~ 12.6V)
 *   分压比: V_bat × R2/(R1+R2) → ADC 量程内 (0~3.3V)
 *   例: R1=33kΩ, R2=10kΩ → 分压比 0.2326, 12.6V → 2.93V
 */

#ifndef PACER_BATTERY_H
#define PACER_BATTERY_H

#include <stdbool.h>

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,     /* 低压警告 */
    BATTERY_CRITICAL,    /* 严重低压，必须降落 */
} battery_status_t;

void battery_init(void);

/**
 * @brief 采样一次电池电压 (在低频任务中调用)
 */
void battery_update(void);

/**
 * @brief 获取当前电池电压 (V)
 */
float battery_get_voltage(void);

/**
 * @brief 获取电池状态
 */
battery_status_t battery_get_status(void);

#endif /* PACER_BATTERY_H */
