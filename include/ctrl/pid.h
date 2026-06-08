/**
 * @file pid.h
 * @brief 通用 PID 控制器
 *
 * 特性:
 *   - 抗积分饱和 (integral clamping)
 *   - 输出限幅
 *   - 可运行时修改参数
 */

#ifndef PACER_PID_H
#define PACER_PID_H

#include <stdbool.h>

typedef struct {
    /* 参数 */
    float kp, ki, kd;

    /* 限幅 */
    float integral_limit;   /* 积分上限 (0=不限) */
    float output_limit;     /* 输出上限 (0=不限) */

    /* 内部状态 */
    float integral;
    float prev_error;
    float output;
} pid_t;

/**
 * @brief 初始化 PID
 * @param pid            控制器
 * @param kp, ki, kd     增益
 * @param integral_limit 积分限幅 (防 windup, 0=不限)
 * @param output_limit   输出限幅 (0=不限)
 */
void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit);

/**
 * @brief 计算一次 PID 输出
 * @param pid   控制器
 * @param error 当前误差 (setpoint - measurement)
 * @param dt    时间间隔 (秒)
 * @return      控制输出
 */
float pid_compute(pid_t *pid, float error, float dt);

/**
 * @brief 重置积分和微分状态
 */
void pid_reset(pid_t *pid);

/**
 * @brief 运行时修改参数 (不影响状态)
 */
void pid_set_gains(pid_t *pid, float kp, float ki, float kd);

#endif /* PACER_PID_H */
