/**
 * @file motor.h
 * @brief 电机驱动 (双路 PWM + 方向控制)
 */

#ifndef PACER_MOTOR_H
#define PACER_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int pin_a;          /* PWM 引脚 A */
    int pin_b;          /* PWM 引脚 B */
    int pwm_range;      /* PWM 分辨率 */
    int pwm_freq;       /* PWM 频率 */
    bool initialized;
} motor_t;

/**
 * @brief 初始化电机驱动
 */
int motor_init(motor_t *m, int pin_a, int pin_b);

/**
 * @brief 设置电机输出
 * @param m     电机
 * @param power 功率 [-pwm_range, +pwm_range]
 *              正=正转, 负=反转, 0=停止
 */
void motor_set(motor_t *m, float power);

/**
 * @brief 电机刹车
 */
void motor_brake(motor_t *m);

/**
 * @brief 关闭电机
 */
void motor_close(motor_t *m);

#endif /* PACER_MOTOR_H */
