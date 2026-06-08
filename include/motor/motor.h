/**
 * @file motor.h
 * @brief 电机驱动统一接口
 *
 * 支持三种驱动方式:
 *   1. GPIO PWM — 有刷电机, IN1/IN2 双通道
 *   2. PCA9685 PWM — 有刷电机, IN1/IN2 双通道
 *   3. PCA9685 ESC — 无刷电调, 单通道舵机信号 (1000~2000μs, 50Hz)
 *
 * 上层只需调 motor_set()，不关心底层。
 */

#ifndef PACER_MOTOR_H
#define PACER_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* 电机类型 */
typedef enum {
    MOTOR_TYPE_DC_GPIO,    /* 有刷电机, GPIO PWM */
    MOTOR_TYPE_DC_PCA9685, /* 有刷电机, PCA9685 PWM (双通道) */
    MOTOR_TYPE_ESC,        /* 无刷电调, PCA9685 舵机信号 (单通道) */
} motor_type_t;

/* 单个电机 */
typedef struct {
    motor_type_t type;     /* 电机类型 */
    int pin_a;             /* 正转通道 / ESC 信号通道 */
    int pin_b;             /* 反转通道 (ESC 模式不用, 填 -1) */
    int pwm_range;         /* PWM 分辨率 (DC: 1000, ESC: 1000) */
    bool inverted;         /* 是否反转方向 */
    bool armed;            /* ESC 是否已解锁 */
    bool ready;
} motor_t;

/**
 * @brief 初始化有刷电机 (DC)
 */
int  motor_init_dc(motor_t *m, motor_type_t type,
                   int pin_a, int pin_b, int pwm_range, bool inverted);

/**
 * @brief 初始化无刷电调 (ESC)
 * @param m         电机
 * @param signal_ch PCA9685 信号通道号
 * @param inverted  是否反转
 */
int  motor_init_esc(motor_t *m, int signal_ch, bool inverted);

/**
 * @brief 设置电机输出
 * @param m     电机
 * @param power -1.0 ~ +1.0 (ESC: -1=反转最大, 0=停止, +1=正转最大)
 *               DC: -pwm_range ~ +pwm_range
 */
void motor_set(motor_t *m, float power);

/**
 * @brief 刹车
 */
void motor_brake(motor_t *m);

/**
 * @brief ESC 解锁 (发送中位信号)
 * @note  某些电调需要先收到 1500μs 才能工作
 */
void motor_esc_arm(motor_t *m);

/**
 * @brief 关闭电机
 */
void motor_deinit(motor_t *m);

#endif /* PACER_MOTOR_H */
