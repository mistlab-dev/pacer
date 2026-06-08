/**
 * @file balance.c
 * @brief 平衡控制器 — 三环串级 PID
 *
 * ┌──────────┐     ┌──────────┐     ┌─────────┐
 * │ 速度环   │────→│ 角度环   │────→│ 电机PWM │
 * │ (外环)   │     │ (内环)   │     │         │
 * └──────────┘     └──────────┘     └─────────┘
 *                         ↑
 *                  ┌──────────┐
 *                  │ 转向环   │──→ 差速
 *                  └──────────┘
 *
 * 物理直觉:
 *   - 车往前倒 → 角度环加大 PWM → 车轮追上去 → 车正了
 *   - 想往前走 → 速度环输出一个前倾角 → 角度环追这个角 → 车就往前跑
 *   - 想转弯   → 转向环输出差速 → 一边快一边慢 → 车就转了
 */

#include "ctrl/balance.h"
#include "app/config.h"

#include <math.h>
#include <stdio.h>

/* ---------- 内部辅助 ---------- */

/** 死区补偿: 克服电机启动摩擦力 */
static float deadzone_compensate(float pwm)
{
    if (pwm > 0.0f && pwm < CFG_MOTOR_DEADZONE) return (float)CFG_MOTOR_DEADZONE;
    if (pwm < 0.0f && pwm > -CFG_MOTOR_DEADZONE) return -(float)CFG_MOTOR_DEADZONE;
    return pwm;
}

/** 限幅 */
static float clampf(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

/* ---------- 公开接口 ---------- */

void balance_init(balance_t *b)
{
    /* 角度环: 核心, 响应要快 */
    pid_init(&b->pid_angle,
             CFG_BAL_ANGLE_KP, CFG_BAL_ANGLE_KI, CFG_BAL_ANGLE_KD,
             10.0f,                                   /* 积分限幅 */
             (float)CFG_MOTOR_MAX_DUTY);               /* 输出 = PWM */

    /* 速度环: 外环, 输出是目标角度 */
    pid_init(&b->pid_speed,
             CFG_BAL_SPEED_KP, CFG_BAL_SPEED_KI, CFG_BAL_SPEED_KD,
             5.0f,
             10.0f);                                    /* 最多倾斜 10° */

    /* 转向环: 辅助, 输出差速 */
    pid_init(&b->pid_steer,
             CFG_BAL_STEER_KP, 0.0f, CFG_BAL_STEER_KD,
             5.0f,
             200.0f);

    b->target_speed    = 0.0f;
    b->target_yaw_rate = 0.0f;
    b->enabled         = false;
    b->output.emergency = false;

    printf("[BALANCE] init  angle(%.1f, %.2f, %.2f)  speed(%.2f, %.3f, %.2f)\n",
           CFG_BAL_ANGLE_KP, CFG_BAL_ANGLE_KI, CFG_BAL_ANGLE_KD,
           CFG_BAL_SPEED_KP, CFG_BAL_SPEED_KI, CFG_BAL_SPEED_KD);
}

const balance_output_t *balance_update(balance_t *b,
                                       const attitude_t *att,
                                       const imu_sample_t *sample,
                                       float dt)
{
    /* 未启用 → 输出零 */
    if (!b->enabled) {
        b->output.motor_left  = 0.0f;
        b->output.motor_right = 0.0f;
        return &b->output;
    }

    /* 紧急停机 → 保持零 */
    if (b->output.emergency) {
        b->output.motor_left  = 0.0f;
        b->output.motor_right = 0.0f;
        return &b->output;
    }

    float roll = att->roll + CFG_BAL_ANGLE_OFFSET;

    /* 安全检查 */
    if (fabsf(roll) > CFG_ANGLE_EMERGENCY_DEG) {
        b->output.emergency  = true;
        b->output.motor_left  = 0.0f;
        b->output.motor_right = 0.0f;
        fprintf(stderr, "[BALANCE] *** EMERGENCY STOP *** roll=%.1f°\n", roll);
        return &b->output;
    }

    /* 角度限幅 (软限制, 不停机) */
    if (fabsf(roll) > CFG_ANGLE_LIMIT_DEG) {
        roll = (roll > 0) ? CFG_ANGLE_LIMIT_DEG : -CFG_ANGLE_LIMIT_DEG;
    }

    /*
     * === 第一环: 速度环 ===
     * 输入: 目标速度 - 实际速度
     * 输出: 目标倾斜角
     *
     * TODO: 实际速度来自编码器, 暂时没有, speed_error = 0
     */
    float speed_error = b->target_speed;  /* 暂无编码器反馈 */
    float roll_target = pid_compute(&b->pid_speed, speed_error, dt);

    /*
     * === 第二环: 角度环 ===
     * 输入: 目标角 - 实际角
     * 输出: 基础 PWM
     */
    float angle_error = roll_target - roll;
    float base_pwm = pid_compute(&b->pid_angle, angle_error, dt);

    /*
     * === 第三环: 转向环 ===
     * 输入: 目标偏航率 - 实际偏航率
     * 输出: 差速
     */
    float steer_error = b->target_yaw_rate - sample->gyro.z;
    float diff_pwm = pid_compute(&b->pid_steer, steer_error, dt);

    /* === 混合输出 === */
    b->output.motor_left  = clampf(base_pwm - diff_pwm, (float)CFG_MOTOR_MAX_DUTY);
    b->output.motor_right = clampf(base_pwm + diff_pwm, (float)CFG_MOTOR_MAX_DUTY);

    /* 死区补偿 */
    b->output.motor_left  = deadzone_compensate(b->output.motor_left);
    b->output.motor_right = deadzone_compensate(b->output.motor_right);

    /* 调试 */
    b->debug.roll         = roll;
    b->debug.roll_target  = roll_target;
    b->debug.gyro_x       = sample->gyro.x;
    b->debug.speed_integral = b->pid_speed.integral;

    return &b->output;
}

void balance_set_speed(balance_t *b, float speed)
{
    b->target_speed = speed;
}

void balance_set_steer(balance_t *b, float yaw_rate)
{
    b->target_yaw_rate = yaw_rate;
}

void balance_enable(balance_t *b, bool on)
{
    if (on && !b->enabled) {
        pid_reset(&b->pid_angle);
        pid_reset(&b->pid_speed);
        pid_reset(&b->pid_steer);
        b->output.emergency = false;
        printf("[BALANCE] enabled\n");
    } else if (!on && b->enabled) {
        b->output.motor_left  = 0.0f;
        b->output.motor_right = 0.0f;
        printf("[BALANCE] disabled\n");
    }
    b->enabled = on;
}

void balance_set_angle_gains(float kp, float ki, float kd)
{
    /* 通过全局配置指针设置 — 在 main 中实现 */
    (void)kp; (void)ki; (void)kd;
}

void balance_set_speed_gains(float kp, float ki, float kd)
{
    (void)kp; (void)ki; (void)kd;
}

void balance_set_angle_offset(float offset_deg)
{
    (void)offset_deg;
}

const balance_debug_t *balance_get_debug(const balance_t *b)
{
    return &b->debug;
}
