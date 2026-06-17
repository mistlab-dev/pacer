/**
 * @file hal_tim.h
 * @brief 硬件 Timer PWM 抽象层 — STM32H743
 *
 * 用高级定时器 TIM1 产生 4 路 ESC PWM 信号 (50Hz, 1000~2000μs)。
 * 替代之前的 PCA9685 外挂。
 */

#ifndef PACER_HAL_TIM_H
#define PACER_HAL_TIM_H

#include <stdint.h>
#include <stdbool.h>

/* ESC 通道索引 */
enum {
    HAL_ESC_CH_FL = 0,
    HAL_ESC_CH_FR = 1,
    HAL_ESC_CH_RL = 2,
    HAL_ESC_CH_RR = 3,
    HAL_ESC_CH_COUNT,
};

/**
 * @brief 初始化 Timer PWM (4通道 ESC 输出)
 * @return 0=成功, -1=失败
 *
 * 配置 TIM1:
 *   - 预分频: CFG_ESC_PWM_PRESCALER → 1MHz 计数
 *   - 周期: CFG_ESC_PWM_PERIOD → 50Hz
 *   - 4 通道 PWM 输出
 *   - 所有通道初始输出 1000μs (ESC 停止/解锁位)
 */
int  hal_tim_pwm_init(void);

/**
 * @brief 设置 ESC 通道脉宽
 * @param channel  HAL_ESC_CH_FL / FR / RL / RR
 * @param pulse_us 脉宽 (μs), 典型范围 1000~2000
 */
void hal_tim_pwm_set_pulse(int channel, uint16_t pulse_us);

/**
 * @brief 停止所有 PWM 输出 (关闭通道)
 */
void hal_tim_pwm_stop_all(void);

/**
 * @brief 启动所有 PWM 通道
 */
void hal_tim_pwm_start_all(void);

#endif /* PACER_HAL_TIM_H */
