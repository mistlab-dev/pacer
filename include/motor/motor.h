/**
 * @file motor.h
 * @brief 电机驱动接口 — ESC (硬件 Timer PWM)
 *
 * STM32H743 版本，专为四旋翼设计。
 *
 * motor_set(channel, power):
 *   power: 0.0 = 最低/停止, 1.0 = 最大油门
 *   对应 ESC 脉宽 1000~2000μs
 */

#ifndef PACER_MOTOR_H
#define PACER_MOTOR_H

#include <stdbool.h>

/* ESC 通道 (与 hal_tim 对应) */
enum {
    MOTOR_FL = 0,   /* 前左  CW  */
    MOTOR_FR = 1,   /* 前右  CCW */
    MOTOR_RL = 2,   /* 后左  CCW */
    MOTOR_RR = 3,   /* 后右  CW  */
    MOTOR_COUNT,
};

/**
 * @brief 初始化电机子系统
 * @return 0=成功
 */
int  motor_init(void);
void motor_deinit(void);

/**
 * @brief 设置单通道油门
 * @param channel MOTOR_FL / FR / RL / RR
 * @param power   0.0~1.0
 */
void motor_set(int channel, float power);

/**
 * @brief 设置所有通道相同油门
 */
void motor_set_all(float power);

/**
 * @brief 全部停止 (输出最低脉宽)
 */
void motor_stop_all(void);

/**
 * @brief ESC 解锁 — 输出最低油门信号
 */
int  motor_arm(void);
void motor_disarm(void);
bool motor_is_armed(void);

#endif /* PACER_MOTOR_H */
