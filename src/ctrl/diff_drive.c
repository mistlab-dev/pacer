/**
 * @file diff_drive.c
 * @brief 四轮差速驱动实现
 *
 * 运动学:
 *   v_left  = throttle - steering
 *   v_right = throttle + steering
 *
 * 差速转向: 一边快一边慢 → 转弯
 *原地转向: throttle=0, steering≠0 → 一边正转一边反转
 */

#include "ctrl/diff_drive.h"
#include "app/config.h"

#include <stdio.h>
#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int diff_drive_init(diff_drive_t *dd, motor_t motors[DIFF_DRIVE_NUM_MOTORS])
{
    for (int i = 0; i < DIFF_DRIVE_NUM_MOTORS; i++) {
        dd->motors[i] = motors[i];
    }

    dd->throttle = 0.0f;
    dd->steering = 0.0f;
    dd->max_power = 1.0f;
    dd->use_pid   = false;
    dd->enabled   = false;

    /* 速度环 PID 暂不启用，等有编码器再开 */
    pid_init(&dd->pid_speed,
             CFG_DIFF_SPEED_KP, CFG_DIFF_SPEED_KI, CFG_DIFF_SPEED_KD,
             5.0f, (float)CFG_MOTOR_PWM_RANGE);

    printf("[DIFF_DRIVE] init  4 motors\n");
    return 0;
}

void diff_drive_set(diff_drive_t *dd, float throttle, float steering)
{
    dd->throttle = clampf(throttle, -1.0f, 1.0f);
    dd->steering = clampf(steering, -1.0f, 1.0f);
}

void diff_drive_update(diff_drive_t *dd, float dt)
{
    if (!dd->enabled) {
        for (int i = 0; i < DIFF_DRIVE_NUM_MOTORS; i++) {
            motor_set(&dd->motors[i], 0.0f);
        }
        return;
    }

    float max_pwm = (float)CFG_MOTOR_PWM_RANGE * dd->max_power;

    /* 差速混合 */
    float left  = (dd->throttle - dd->steering) * max_pwm;
    float right = (dd->throttle + dd->steering) * max_pwm;

    /* 限幅 */
    left  = clampf(left,  -max_pwm, max_pwm);
    right = clampf(right, -max_pwm, max_pwm);

    /* 左侧电机 (前左 + 后左) */
    motor_set(&dd->motors[MOTOR_FL], left);
    motor_set(&dd->motors[MOTOR_RL], left);

    /* 右侧电机 (前右 + 后右) */
    motor_set(&dd->motors[MOTOR_FR], right);
    motor_set(&dd->motors[MOTOR_RR], right);

    (void)dt; /* 开环模式不用 dt */
}

void diff_drive_brake(diff_drive_t *dd)
{
    for (int i = 0; i < DIFF_DRIVE_NUM_MOTORS; i++) {
        motor_brake(&dd->motors[i]);
    }
}

void diff_drive_enable(diff_drive_t *dd, bool on)
{
    if (on && !dd->enabled) {
        dd->throttle = 0.0f;
        dd->steering = 0.0f;
        printf("[DIFF_DRIVE] enabled\n");
    } else if (!on && dd->enabled) {
        diff_drive_brake(dd);
        printf("[DIFF_DRIVE] disabled\n");
    }
    dd->enabled = on;
}

void diff_drive_deinit(diff_drive_t *dd)
{
    diff_drive_brake(dd);
    for (int i = 0; i < DIFF_DRIVE_NUM_MOTORS; i++) {
        motor_deinit(&dd->motors[i]);
    }
    printf("[DIFF_DRIVE] deinit\n");
}
