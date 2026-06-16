/**
 * @file flight_phase.c
 * @brief 飞行阶段状态机实现
 *
 * 油门控制策略:
 *
 *   IDLE     → 油门 = 0, 电机 idle
 *   TAKEOFF  → 油门从 0 爬升到 takeoff_throttle, 爬到后切到 hover_throttle
 *   HOVER    → 油门 = hover_throttle, 姿态由 PID 自稳
 *   LANDING  → 油门从当前值缓慢降到 0, 触地后切 GROUNDED
 *   GROUNDED → 油门 = 0
 *
 * 安全:
 *   倾斜超过 emergency → 自动紧急停机
 *   HOVER 中倾斜超 max_drift → 自动转 LANDING
 */

#include "app/flight_phase.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============ 内部 ============ */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float ramp_toward(float current, float target, float rate, float dt)
{
    if (current < target) {
        current += rate * dt;
        if (current > target) current = target;
    } else if (current > target) {
        current -= rate * dt;
        if (current < target) current = target;
    }
    return current;
}

static float tilt_magnitude(float roll, float pitch)
{
    return sqrtf(roll * roll + pitch * pitch);
}

static void emit_event(flight_phase_t *fp, flight_phase_event_t evt)
{
    fp->last_event = evt;
}

/* ============ 公开接口 ============ */

void flight_phase_init(flight_phase_t *fp, const flight_phase_config_t *cfg)
{
    memset(fp, 0, sizeof(*fp));
    fp->phase = PHASE_IDLE;
    fp->cfg   = *cfg;
    fp->current_throttle = 0.0f;
    fp->target_throttle  = 0.0f;
    fp->ramp_active      = false;
    fp->last_event       = PHASE_EVT_NONE;
    printf("[PHASE] init phase=IDLE hover=%.0f%% takeoff=%.0f%%\n",
           fp->cfg.hover_throttle * 100.0f,
           fp->cfg.takeoff_throttle * 100.0f);
}

int flight_phase_request_takeoff(flight_phase_t *fp)
{
    if (fp->phase != PHASE_IDLE) {
        fprintf(stderr, "[PHASE] takeoff rejected: 当前 %s\n",
                flight_phase_name(fp->phase));
        return -1;
    }

    fp->phase = PHASE_TAKEOFF;
    fp->current_throttle = 0.0f;
    fp->target_throttle  = fp->cfg.takeoff_throttle;
    fp->ramp_active      = true;
    fp->phase_time       = 0.0f;
    fp->last_event       = PHASE_EVT_TAKEOFF_START;
    printf("[PHASE] *** TAKEOFF *** 油门爬升到 %.0f%%\n",
           fp->cfg.takeoff_throttle * 100.0f);
    return 0;
}

int flight_phase_request_landing(flight_phase_t *fp)
{
    if (fp->phase != PHASE_HOVER && fp->phase != PHASE_TAKEOFF) {
        fprintf(stderr, "[PHASE] landing rejected: 当前 %s\n",
                flight_phase_name(fp->phase));
        return -1;
    }

    fp->phase = PHASE_LANDING;
    fp->target_throttle = 0.0f;
    fp->ramp_active     = true;
    fp->landing_timer   = 0.0f;
    fp->phase_time      = 0.0f;
    fp->last_event      = PHASE_EVT_LAND_START;
    printf("[PHASE] *** LANDING *** 开始降落\n");
    return 0;
}

void flight_phase_emergency(flight_phase_t *fp)
{
    printf("[PHASE] *** EMERGENCY *** 立即停机!\n");
    fp->phase           = PHASE_GROUNDED;
    fp->current_throttle = 0.0f;
    fp->target_throttle  = 0.0f;
    fp->ramp_active      = false;
    fp->last_event       = PHASE_EVT_EMERGENCY;
}

void flight_phase_reset(flight_phase_t *fp)
{
    fp->phase            = PHASE_IDLE;
    fp->current_throttle = 0.0f;
    fp->target_throttle  = 0.0f;
    fp->ramp_active      = false;
    fp->phase_time       = 0.0f;
    fp->landing_timer    = 0.0f;
    fp->last_event       = PHASE_EVT_NONE;
    printf("[PHASE] reset → IDLE\n");
}

float flight_phase_update(flight_phase_t *fp,
                           float roll_deg,
                           float pitch_deg,
                           float dt)
{
    fp->phase_time += dt;
    float tilt = tilt_magnitude(roll_deg, pitch_deg);

    /* 紧急倾斜保护 — 任何飞行阶段 */
    if ((fp->phase == PHASE_TAKEOFF || fp->phase == PHASE_HOVER) &&
        tilt > fp->cfg.tilt_emergency_deg) {
        flight_phase_emergency(fp);
        return 0.0f;
    }

    /* 悬停中倾斜过大 → 自动降落 */
    if (fp->phase == PHASE_HOVER &&
        tilt > fp->cfg.hover_max_drift_deg) {
        printf("[PHASE] 悬停倾斜 %.1f° > %.1f° → 自动降落\n",
               tilt, fp->cfg.hover_max_drift_deg);
        flight_phase_request_landing(fp);
    }

    switch (fp->phase) {

    /* ---- 地面待命 ---- */
    case PHASE_IDLE:
        fp->current_throttle = 0.0f;
        break;

    /* ---- 起飞: 油门爬坡 ---- */
    case PHASE_TAKEOFF:
        if (fp->ramp_active) {
            fp->current_throttle = ramp_toward(
                fp->current_throttle,
                fp->target_throttle,
                fp->cfg.takeoff_ramp_rate,
                dt);

            /* 到达目标油门 → 切到悬停 */
            if (fabsf(fp->current_throttle - fp->target_throttle) < 0.01f) {
                fp->phase = PHASE_HOVER;
                fp->target_throttle = fp->cfg.hover_throttle;
                fp->hover_entry_roll  = roll_deg;
                fp->hover_entry_pitch = pitch_deg;
                fp->phase_time = 0.0f;
                fp->last_event = PHASE_EVT_TAKEOFF_DONE;
                fp->total_flight_time += 0; /* will accumulate in HOVER */
                printf("[PHASE] *** HOVER *** 悬停油门 %.0f%%\n",
                       fp->cfg.hover_throttle * 100.0f);
            }
        }
        fp->total_flight_time += dt;
        break;

    /* ---- 悬停 ---- */
    case PHASE_HOVER:
        fp->current_throttle = fp->cfg.hover_throttle;
        fp->total_flight_time += dt;
        break;

    /* ---- 降落: 油门缓降 ---- */
    case PHASE_LANDING: {
        fp->landing_timer += dt;
        fp->total_flight_time += dt;

        if (fp->ramp_active) {
            fp->current_throttle = ramp_toward(
                fp->current_throttle,
                0.0f,
                fp->cfg.landing_descent_rate,
                dt);
        }

        /* 油门降到最低 → 触地 */
        if (fp->current_throttle <= fp->cfg.landing_throttle_min + 0.01f) {
            fp->phase = PHASE_GROUNDED;
            fp->current_throttle = 0.0f;
            fp->ramp_active = false;
            fp->last_event = PHASE_EVT_TOUCHDOWN;
            printf("[PHASE] *** TOUCHDOWN *** 着陆完成 (飞行 %.1fs)\n",
                   fp->total_flight_time);
        }

        /* 超时保护: 降落超过 10s 强制触地 */
        if (fp->landing_timer > 10.0f) {
            printf("[PHASE] 降落超时, 强制触地\n");
            fp->phase = PHASE_GROUNDED;
            fp->current_throttle = 0.0f;
            fp->last_event = PHASE_EVT_TOUCHDOWN;
        }
        break;
    }

    /* ---- 已着陆 ---- */
    case PHASE_GROUNDED:
        fp->current_throttle = 0.0f;
        break;
    }

    return clampf(fp->current_throttle, 0.0f, 1.0f);
}

flight_phase_state_t flight_phase_get(const flight_phase_t *fp)
{
    return fp->phase;
}

const char *flight_phase_name(flight_phase_state_t phase)
{
    switch (phase) {
        case PHASE_IDLE:     return "IDLE";
        case PHASE_TAKEOFF:  return "TAKEOFF";
        case PHASE_HOVER:    return "HOVER";
        case PHASE_LANDING:  return "LANDING";
        case PHASE_GROUNDED: return "GROUNDED";
    }
    return "?";
}

flight_phase_event_t flight_phase_last_event(const flight_phase_t *fp)
{
    return fp->last_event;
}

bool flight_phase_in_flight(const flight_phase_t *fp)
{
    return fp->phase == PHASE_TAKEOFF ||
           fp->phase == PHASE_HOVER   ||
           fp->phase == PHASE_LANDING;
}
