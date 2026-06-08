/**
 * @file pid.c
 * @brief PID 控制器实现 (抗积分饱和 + 微分滤波)
 */

#include "control/pid.h"
#include <math.h>

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->enabled = true;
}

float pid_update(pid_t *pid, float error, float dt)
{
    if (!pid->enabled) return 0;

    /* 比例项 */
    float p_term = pid->kp * error;

    /* 积分项 (带限幅防 windup) */
    pid->integral += error * dt;
    if (pid->integral_limit > 0) {
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        else if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    /* 微分项 (基于误差变化率) */
    float d_term = 0;
    if (dt > 0.0001f) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;

    /* 总输出 */
    pid->output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (pid->output_limit > 0) {
        if (pid->output > pid->output_limit)
            pid->output = pid->output_limit;
        else if (pid->output < -pid->output_limit)
            pid->output = -pid->output_limit;
    }

    return pid->output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}

void pid_set_params(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
