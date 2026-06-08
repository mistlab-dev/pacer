/**
 * @file diff_drive.c
 * @brief 四轮差速驱动实现
 *
 * 运动学:
 *   v_left  = throttle - steering
 *   v_right = throttle + steering
 *
 * 自动适配 DC 和 ESC 两种电机:
 *   DC  模式: motor_set 传 -pwm_range ~ +pwm_range
 *   ESC 模式: motor_set 传 -1.0 ~ +1.0
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

/* 判断是否全部为 ESC 模式 */
static bool all_esc(const diff_drive_t *dd)
{
    for (int i = 0; i < DIFF_DRIVE_NUM_MOTORS; i++) {
        if (dd->motors[i].type != MOTOR_TYPE_ESC) return false;
    }
    return true;
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

    pid_init(&dd->pid_speed,
             CFG_DIFF_SPEED_KP, CFG_DIFF_SPEED_KI, CFG_DIFF_SPEED_KD,
             5.0f, 1.0f);

    printf("[DIFF_DRIVE] init mode=%s\n", all_esc(dd) ? "ESC" : "DC");
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

    bool is_esc = all_esc(dd);

    /* 差速混合: left/right 都是 -1~+1 */
    float left  = dd->throttle - dd->steering;
    float right = dd->throttle + dd->steering;

    left  = clampf(left,  -1.0f, 1.0f) * dd->max_power;
    right = clampf(right, -1.0f, 1.0f) * dd->max_power;

    if (!is_esc) {
        /* DC 模式: 乘以 pwm_range */
        float max_pwm = (float)CFG_MOTOR_PWM_RANGE;
        left  *= max_pwm;
        right *= max_pwm;
    }

    /* 左侧电机 */
    motor_set(&dd->motors[MOTOR_FL], left);
    motor_set(&dd->motors[MOTOR_RL], left);

    /* 右侧电机 */
    motor_set(&dd->motors[MOTOR_FR], right);
    motor_set(&dd->motors[MOTOR_RR], right);

    (void)dt;
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
