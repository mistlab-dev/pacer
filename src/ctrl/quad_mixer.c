/**
 * @file quad_mixer.c
 * @brief 四旋翼混控器实现
 *
 * 核心公式 ("X" 型混控):
 *   FL = throttle - pitch + roll - yaw
 *   FR = throttle - pitch - roll + yaw
 *   RL = throttle + pitch + roll + yaw
 *   RR = throttle + pitch - roll - yaw
 */

#include "ctrl/quad_mixer.h"
#include <stdio.h>
#include <string.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int quad_mixer_init(quad_mixer_t *m, const mixer_config_t *cfg)
{
    memset(m, 0, sizeof(*m));
    m->cfg    = *cfg;
    m->armed  = false;
    m->enabled = false;

    printf("[MIXER] init hover=%.0f%% range=[%.0f%%, %.0f%%]\n",
           m->cfg.throttle_hover * 100.0f,
           m->cfg.throttle_min * 100.0f,
           m->cfg.throttle_max * 100.0f);
    return 0;
}

void quad_mixer_arm(quad_mixer_t *m)
{
    m->armed  = true;
    m->enabled = true;
    printf("[MIXER] ARMED\n");
}

void quad_mixer_disarm(quad_mixer_t *m)
{
    m->armed  = false;
    m->enabled = false;
    printf("[MIXER] DISARMED\n");
}

mixer_output_t quad_mixer_update(quad_mixer_t *m,
                                  float throttle,
                                  float roll,
                                  float pitch,
                                  float yaw)
{
    mixer_output_t out = {{0}};

    if (!m->armed || !m->enabled) {
        return out;
    }

    /* 限幅姿态校正量 */
    roll  = clampf(roll,  -m->cfg.max_roll_rate,  m->cfg.max_roll_rate);
    pitch = clampf(pitch, -m->cfg.max_pitch_rate, m->cfg.max_pitch_rate);
    yaw   = clampf(yaw,   -m->cfg.max_yaw_rate,   m->cfg.max_yaw_rate);

    /* 油门范围 */
    throttle = clampf(throttle, 0.0f, 1.0f);

    /* "X" 型混控 */
    out.motor[QUAD_FL] = throttle - pitch + roll  - yaw;
    out.motor[QUAD_FR] = throttle - pitch - roll  + yaw;
    out.motor[QUAD_RL] = throttle + pitch + roll  + yaw;
    out.motor[QUAD_RR] = throttle + pitch - roll  - yaw;

    /* 限幅到 0~1 (不能有负油门) */
    for (int i = 0; i < QUAD_NUM_MOTORS; i++) {
        out.motor[i] = clampf(out.motor[i], m->cfg.throttle_min, m->cfg.throttle_max);
    }

    return out;
}

void quad_mixer_stop(quad_mixer_t *m)
{
    m->enabled = false;
    printf("[MIXER] STOP\n");
}

void quad_mixer_deinit(quad_mixer_t *m)
{
    m->armed  = false;
    m->enabled = false;
    printf("[MIXER] deinit\n");
}

bool quad_mixer_is_armed(const quad_mixer_t *m)
{
    return m->armed;
}
