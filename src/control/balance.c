/**
 * @file balance.c
 * @brief 平衡控制实现 — 三环串级 PID
 *
 * 核心逻辑:
 *   1. 速度环: 速度误差 -> 目标倾斜角 (前倾加速/后倾减速)
 *   2. 角度环: (目标角 - 实际角) -> 电机 PWM (保持平衡)
 *   3. 转向环: 目标偏航率 -> 左右差速
 */

#include "control/balance.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

void balance_init(balance_t *bal)
{
    /* 角度环 — 平衡核心 */
    pid_init(&bal->angle_pid,
             BALANCE_KP, BALANCE_KI, BALANCE_KD,
             10.0f,           /* 积分限幅 */
             MOTOR_PWM_RANGE); /* 输出限幅 = PWM 范围 */

    /* 速度环 */
    pid_init(&bal->speed_pid,
             SPEED_KP, SPEED_KI, SPEED_KD,
             5.0f,
             SPEED_MAX_OUTPUT);

    /* 转向环 */
    pid_init(&bal->steer_pid,
             STEER_KP, 0, STEER_KD,
             5.0f,
             200.0f);

    bal->motor_left = 0;
    bal->motor_right = 0;
    bal->running = false;
    bal->emergency = false;
    bal->target_speed = 0;
    bal->target_steer = 0;

    printf("[BAL] Balance controller initialized\n");
    printf("[BAL]   Angle PID: Kp=%.1f Ki=%.2f Kd=%.2f\n",
           BALANCE_KP, BALANCE_KI, BALANCE_KD);
    printf("[BAL]   Speed PID: Kp=%.1f Ki=%.2f Kd=%.2f\n",
           SPEED_KP, SPEED_KI, SPEED_KD);
}

void balance_update(balance_t *bal, const imu_data_t *imu,
                    const attitude_t *att, float dt)
{
    if (!bal->running || bal->emergency) {
        bal->motor_left = 0;
        bal->motor_right = 0;
        return;
    }

    float roll = att->roll + BALANCE_ANGLE_OFFSET;

    /* 紧急停机检查 */
    if (fabsf(roll) > ANGLE_EMERGENCY) {
        balance_emergency(bal);
        return;
    }

    /* 角度限制 */
    if (fabsf(roll) > ANGLE_MAX) {
        /* 超出范围但还没到紧急停机 — 减弱输出 */
        roll = (roll > 0) ? ANGLE_MAX : -ANGLE_MAX;
    }

    /* === 速度环 === */
    /* 没有编码器时, 速度环暂时用 0 目标 */
    float speed_error = bal->target_speed; /* TODO: 反馈编码器速度 */
    float angle_target = pid_update(&bal->speed_pid, speed_error, dt);

    /* === 角度环 === */
    float angle_error = angle_target - roll;
    bal->debug_angle = roll;
    bal->debug_angle_target = angle_target;
    bal->debug_speed_error = speed_error;

    float base_pwm = pid_update(&bal->angle_pid, angle_error, dt);

    /* === 转向环 === */
    float steer_error = bal->target_steer - imu->gyro[2]; /* yaw rate */
    float diff = pid_update(&bal->steer_pid, steer_error, dt);

    /* === 混合输出 === */
    bal->motor_left  = base_pwm - diff;  /* 左电机 */
    bal->motor_right = base_pwm + diff;  /* 右电机 (镜像) */

    /* 限幅 */
    float limit = (float)MOTOR_MAX_DUTY;
    if (bal->motor_left > limit)  bal->motor_left = limit;
    if (bal->motor_left < -limit) bal->motor_left = -limit;
    if (bal->motor_right > limit)  bal->motor_right = limit;
    if (bal->motor_right < -limit) bal->motor_right = -limit;

/* 死区补偿 — 克服电机启动摩擦 */
static float apply_deadzone(float pwm)
{
    if (pwm > 0 && pwm < MOTOR_MIN_DUTY) return MOTOR_MIN_DUTY;
    if (pwm < 0 && pwm > -MOTOR_MIN_DUTY) return -MOTOR_MIN_DUTY;
    return pwm;
}

    bal->motor_left = apply_deadzone(bal->motor_left);
    bal->motor_right = apply_deadzone(bal->motor_right);
}

void balance_set_speed(balance_t *bal, float speed)
{
    bal->target_speed = speed;
}

void balance_set_steer(balance_t *bal, float steer)
{
    bal->target_steer = steer;
}

void balance_start(balance_t *bal)
{
    bal->running = true;
    bal->emergency = false;
    pid_reset(&bal->angle_pid);
    pid_reset(&bal->speed_pid);
    pid_reset(&bal->steer_pid);
    printf("[BAL] Started\n");
}

void balance_stop(balance_t *bal)
{
    bal->running = false;
    bal->motor_left = 0;
    bal->motor_right = 0;
    printf("[BAL] Stopped\n");
}

void balance_emergency(balance_t *bal)
{
    bal->emergency = true;
    bal->running = false;
    bal->motor_left = 0;
    bal->motor_right = 0;
    fprintf(stderr, "[BAL] *** EMERGENCY STOP ***\n");
}
