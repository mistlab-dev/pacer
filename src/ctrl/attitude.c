/**
 * @file attitude.c
 * @brief 四旋翼姿态控制器 — 串级 PID 实现
 *
 * 角速度环 (内环):
 *   error = target_rate - gyro_rate
 *   output = PID(error) → 混控器校正量
 *
 * 角度环 (外环, 自稳模式):
 *   error = target_angle - current_angle
 *   output = PID(error) → 角速度环的 target_rate
 *
 * 自稳模式: 遥控杆 → 目标角度 → 角度环 → 目标角速度 → 角速度环 → 混控器
 * 角速度模式: 遥控杆 → 目标角速度 → 角速度环 → 混控器
 */

#include "ctrl/attitude.h"
#include "app/config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---------- 内部辅助 ---------- */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------- 公开接口 ---------- */

void attitude_init(attitude_ctrl_t *ac)
{
    memset(ac, 0, sizeof(*ac));

    /* 角速度环 PID (内环 — 核心, 要快) */
    pid_init(&ac->pid_roll_rate,
             CFG_QUAD_ROLL_RATE_KP,
             CFG_QUAD_ROLL_RATE_KI,
             CFG_QUAD_ROLL_RATE_KD,
             0.5f,    /* 积分限幅 */
             1.0f);   /* 输出限幅 -1~+1 */

    pid_init(&ac->pid_pitch_rate,
             CFG_QUAD_PITCH_RATE_KP,
             CFG_QUAD_PITCH_RATE_KI,
             CFG_QUAD_PITCH_RATE_KD,
             0.5f,
             1.0f);

    pid_init(&ac->pid_yaw_rate,
             CFG_QUAD_YAW_RATE_KP,
             CFG_QUAD_YAW_RATE_KI,
             CFG_QUAD_YAW_RATE_KD,
             0.3f,
             1.0f);

    /* 角度环 PID (外环 — 自稳模式用) */
    pid_init(&ac->pid_roll_angle,
             CFG_QUAD_ROLL_ANGLE_KP,
             CFG_QUAD_ROLL_ANGLE_KI,
             CFG_QUAD_ROLL_ANGLE_KD,
             10.0f,    /* 积分限幅 */
             (float)CFG_QUAD_MAX_ROLL_RATE);   /* 输出=目标角速度 */

    pid_init(&ac->pid_pitch_angle,
             CFG_QUAD_PITCH_ANGLE_KP,
             CFG_QUAD_PITCH_ANGLE_KI,
             CFG_QUAD_PITCH_ANGLE_KD,
             10.0f,
             (float)CFG_QUAD_MAX_PITCH_RATE);

    ac->mode    = ATT_MODE_RATE;
    ac->enabled = false;

    printf("[ATTITUDE] init mode=%s\n",
           ac->mode == ATT_MODE_RATE ? "RATE" : "ANGLE");
    printf("  rate: roll(%.2f,%.3f,%.2f) pitch(%.2f,%.3f,%.2f) yaw(%.2f,%.3f,%.2f)\n",
           CFG_QUAD_ROLL_RATE_KP, CFG_QUAD_ROLL_RATE_KI, CFG_QUAD_ROLL_RATE_KD,
           CFG_QUAD_PITCH_RATE_KP, CFG_QUAD_PITCH_RATE_KI, CFG_QUAD_PITCH_RATE_KD,
           CFG_QUAD_YAW_RATE_KP, CFG_QUAD_YAW_RATE_KI, CFG_QUAD_YAW_RATE_KD);
}

const attitude_output_t *attitude_update(attitude_ctrl_t *ac,
                                          const attitude_t *att,
                                          const imu_sample_t *sample,
                                          float dt)
{
    if (!ac->enabled) {
        ac->output.throttle = 0.0f;
        ac->output.roll     = 0.0f;
        ac->output.pitch    = 0.0f;
        ac->output.yaw      = 0.0f;
        return &ac->output;
    }

    float roll_rate  = sample->gyro.x;   /* 度/s */
    float pitch_rate = sample->gyro.y;
    float yaw_rate   = sample->gyro.z;

    float target_roll_rate  = ac->target_roll;
    float target_pitch_rate = ac->target_pitch;
    float target_yaw_rate   = ac->target_yaw_rate;

    if (ac->mode == ATT_MODE_ANGLE) {
        /* 外环: 角度 PID → 目标角速度 */
        float roll_angle_error  = ac->target_roll  - att->roll;
        float pitch_angle_error = ac->target_pitch - att->pitch;

        target_roll_rate  = pid_compute(&ac->pid_roll_angle,  roll_angle_error,  dt);
        target_pitch_rate = pid_compute(&ac->pid_pitch_angle, pitch_angle_error, dt);
    }

    /* 内环: 角速度 PID → 混控校正量 */
    float roll_error  = target_roll_rate  - roll_rate;
    float pitch_error = target_pitch_rate - pitch_rate;
    float yaw_error   = target_yaw_rate   - yaw_rate;

    float roll_out  = pid_compute(&ac->pid_roll_rate,  roll_error,  dt);
    float pitch_out = pid_compute(&ac->pid_pitch_rate, pitch_error, dt);
    float yaw_out   = pid_compute(&ac->pid_yaw_rate,   yaw_error,   dt);

    /* 输出 */
    ac->output.throttle = ac->target_throttle;
    ac->output.roll     = clampf(roll_out,  -1.0f, 1.0f);
    ac->output.pitch    = clampf(pitch_out, -1.0f, 1.0f);
    ac->output.yaw      = clampf(yaw_out,   -1.0f, 1.0f);

    /* 调试 */
    ac->debug.roll         = att->roll;
    ac->debug.pitch        = att->pitch;
    ac->debug.roll_rate    = roll_rate;
    ac->debug.pitch_rate   = pitch_rate;
    ac->debug.yaw_rate     = yaw_rate;
    ac->debug.roll_target  = ac->target_roll;
    ac->debug.pitch_target = ac->target_pitch;
    ac->debug.yaw_target   = ac->target_yaw_rate;

    return &ac->output;
}

void attitude_set_input(attitude_ctrl_t *ac,
                         float throttle,
                         float roll_stick,
                         float pitch_stick,
                         float yaw_stick)
{
    ac->target_throttle = clampf(throttle, 0.0f, 1.0f);
    ac->target_yaw_rate = yaw_stick * (float)CFG_QUAD_MAX_YAW_RATE;

    if (ac->mode == ATT_MODE_ANGLE) {
        /* 遥控杆 → 目标角度 */
        ac->target_roll  = roll_stick  * (float)CFG_QUAD_MAX_ANGLE;
        ac->target_pitch = pitch_stick * (float)CFG_QUAD_MAX_ANGLE;
    } else {
        /* 遥控杆 → 目标角速度 */
        ac->target_roll  = roll_stick  * (float)CFG_QUAD_MAX_ROLL_RATE;
        ac->target_pitch = pitch_stick * (float)CFG_QUAD_MAX_PITCH_RATE;
    }
}

void attitude_set_mode(attitude_ctrl_t *ac, att_mode_t mode)
{
    if (ac->mode != mode) {
        pid_reset(&ac->pid_roll_angle);
        pid_reset(&ac->pid_pitch_angle);
        pid_reset(&ac->pid_roll_rate);
        pid_reset(&ac->pid_pitch_rate);
        pid_reset(&ac->pid_yaw_rate);
        ac->mode = mode;
        printf("[ATTITUDE] mode=%s\n", mode == ATT_MODE_RATE ? "RATE" : "ANGLE");
    }
}

void attitude_enable(attitude_ctrl_t *ac, bool on)
{
    if (on && !ac->enabled) {
        pid_reset(&ac->pid_roll_rate);
        pid_reset(&ac->pid_pitch_rate);
        pid_reset(&ac->pid_yaw_rate);
        pid_reset(&ac->pid_roll_angle);
        pid_reset(&ac->pid_pitch_angle);
        ac->output.throttle = 0.0f;
        ac->output.roll     = 0.0f;
        ac->output.pitch    = 0.0f;
        ac->output.yaw      = 0.0f;
        printf("[ATTITUDE] enabled\n");
    } else if (!on && ac->enabled) {
        ac->output.throttle = 0.0f;
        ac->output.roll     = 0.0f;
        ac->output.pitch    = 0.0f;
        ac->output.yaw      = 0.0f;
        printf("[ATTITUDE] disabled\n");
    }
    ac->enabled = on;
}

void attitude_set_rate_gains(float roll_kp, float roll_ki, float roll_kd,
                              float pitch_kp, float pitch_ki, float pitch_kd,
                              float yaw_kp, float yaw_ki, float yaw_kd)
{
    (void)roll_kp; (void)roll_ki; (void)roll_kd;
    (void)pitch_kp; (void)pitch_ki; (void)pitch_kd;
    (void)yaw_kp; (void)yaw_ki; (void)yaw_kd;
}

const attitude_debug_t *attitude_get_debug(const attitude_ctrl_t *ac)
{
    return &ac->debug;
}
