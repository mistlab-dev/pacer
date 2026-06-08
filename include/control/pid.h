/**
 * @file pid.h
 * @brief PID 控制器 — 平衡/速度/转向三环控制
 */

#ifndef PACER_PID_H
#define PACER_PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
    float integral_limit;   /* 积分限幅 (防 windup) */
    float output_limit;     /* 输出限幅 */
    bool  enabled;
} pid_t;

/**
 * @brief 初始化 PID
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);

/**
 * @brief PID 计算
 * @param pid  PID 控制器
 * @param error 误差
 * @param dt   时间间隔 (秒)
 * @return 控制输出
 */
float pid_update(pid_t *pid, float error, float dt);

/**
 * @brief 重置 PID 状态
 */
void pid_reset(pid_t *pid);

/**
 * @brief 设置 PID 参数
 */
void pid_set_params(pid_t *pid, float kp, float ki, float kd);

#endif /* PACER_PID_H */
