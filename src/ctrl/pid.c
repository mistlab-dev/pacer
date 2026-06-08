/**
 * @file pid.c
 * @brief 通用 PID 控制器
 *
 * P — 比例: 当前误差有多大 → 即时响应
 * I — 积分: 累积误差有多大 → 消除稳态偏差
 * D — 微分: 误差变化有多快 → 预判、阻尼
 *
 * 防止积分饱和: integral 做限幅, 避免 windup
 */

#include "ctrl/pid.h"
#include <math.h>

void pid_init(pacer_pid_t *pid,
              float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

float pid_compute(pacer_pid_t *pid, float error, float dt)
{
    /* P */
    float p = pid->kp * error;

    /* I (梯形积分 + 限幅) */
    pid->integral += error * dt;
    if (pid->integral_limit > 0.0f) {
        if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }
    float i = pid->ki * pid->integral;

    /* D (后向差分) */
    float d = 0.0f;
    if (dt > 1e-6f) {
        d = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    /* 合计 */
    float out = p + i + d;

    /* 输出限幅 */
    if (pid->output_limit > 0.0f) {
        if (out >  pid->output_limit) out =  pid->output_limit;
        if (out < -pid->output_limit) out = -pid->output_limit;
    }

    pid->output = out;
    return out;
}

void pid_reset(pacer_pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

void pid_set_gains(pacer_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
