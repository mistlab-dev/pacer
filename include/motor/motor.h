/**
 * @file motor.h
 * @brief 电机驱动统一接口
 *
 * 抽象双路 PWM 电机，上层只需调 set_speed()。
 */

#ifndef PACER_MOTOR_H
#define PACER_MOTOR_H

#include <stdbool.h>

/* 单个电机 */
typedef struct {
    int pin_a;       /* PWM 引脚 A (正转) */
    int pin_b;       /* PWM 引脚 B (反转) */
    int pwm_range;   /* PWM 分辨率 (如 1000) */
    bool inverted;   /* 是否反转方向 */
    bool ready;
} motor_t;

/**
 * @brief 初始化电机
 * @param m         电机
 * @param pin_a     正转 PWM 引脚
 * @param pin_b     反转 PWM 引脚
 * @param pwm_range PWM 分辨率
 * @param inverted  是否反转 (物理安装方向相反时)
 * @return 0=成功
 */
int  motor_init(motor_t *m, int pin_a, int pin_b, int pwm_range, bool inverted);

/**
 * @brief 设置电机输出
 * @param m     电机
 * @param power -pwm_range ~ +pwm_range, 正=正转, 负=反转, 0=滑行
 */
void motor_set(motor_t *m, float power);

/**
 * @brief 刹车 (两引脚拉高)
 */
void motor_brake(motor_t *m);

/**
 * @brief 关闭电机
 */
void motor_deinit(motor_t *m);

#endif /* PACER_MOTOR_H */
