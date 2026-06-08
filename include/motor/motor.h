/**
 * @file motor.h
 * @brief 电机驱动统一接口
 *
 * 支持两种 PWM 源:
 *   - GPIO 直连 (树莓派原生 PWM)
 *   - PCA9685 I2C PWM 扩展板
 *
 * 上层只需调 motor_set()，不关心底层 PWM 来源。
 */

#ifndef PACER_MOTOR_H
#define PACER_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* PWM 源类型 */
typedef enum {
    MOTOR_PWM_GPIO,      /* 树莓派原生 GPIO PWM */
    MOTOR_PWM_PCA9685,   /* PCA9685 I2C PWM */
} motor_pwm_src_t;

/* 单个电机 */
typedef struct {
    motor_pwm_src_t pwm_src;  /* PWM 来源 */
    int pin_a;                /* 正转通道 (GPIO pin 或 PCA9685 channel) */
    int pin_b;                /* 反转通道 */
    int pwm_range;            /* PWM 分辨率 (如 1000) */
    bool inverted;            /* 是否反转方向 */
    bool ready;
} motor_t;

/**
 * @brief 初始化电机
 * @param m         电机
 * @param pwm_src   PWM 来源
 * @param pin_a     正转通道
 * @param pin_b     反转通道
 * @param pwm_range PWM 分辨率
 * @param inverted  是否反转
 * @return 0=成功
 */
int  motor_init(motor_t *m, motor_pwm_src_t pwm_src,
                int pin_a, int pin_b, int pwm_range, bool inverted);

/**
 * @brief 设置电机输出
 * @param m     电机
 * @param power -pwm_range ~ +pwm_range, 正=正转, 负=反转, 0=滑行
 */
void motor_set(motor_t *m, float power);

/**
 * @brief 刹车 (两通道拉满)
 */
void motor_brake(motor_t *m);

/**
 * @brief 关闭电机
 */
void motor_deinit(motor_t *m);

#endif /* PACER_MOTOR_H */
