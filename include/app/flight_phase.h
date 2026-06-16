/**
 * @file flight_phase.h
 * @brief 飞行阶段状态机 — IDLE → TAKEOFF → HOVER → LANDING
 *
 * 起飞: 逐步提升油门到悬停值，保持水平
 * 悬停: 自稳模式，油门锁定在悬停值
 * 降落: 逐步降低油门直到触地
 */

#ifndef PACER_FLIGHT_PHASE_H
#define PACER_FLIGHT_PHASE_H

#include <stdbool.h>
#include <stdint.h>

/* 飞行阶段 (枚举) */
typedef enum {
    PHASE_IDLE     = 0,   /* 地面待命 */
    PHASE_TAKEOFF  = 1,   /* 起飞爬升 */
    PHASE_HOVER    = 2,   /* 定高悬停 */
    PHASE_LANDING  = 3,   /* 降落 */
    PHASE_GROUNDED = 4,   /* 已着陆，等待上锁 */
} flight_phase_state_t;

/* 飞行阶段事件 */
typedef enum {
    PHASE_EVT_NONE          = 0,
    PHASE_EVT_TAKEOFF_START = 1,   /* 开始起飞 */
    PHASE_EVT_TAKEOFF_DONE  = 2,   /* 起飞完成 → 悬停 */
    PHASE_EVT_LAND_START    = 3,   /* 开始降落 */
    PHASE_EVT_TOUCHDOWN     = 4,   /* 触地 */
    PHASE_EVT_EMERGENCY     = 5,   /* 紧急停机 */
} flight_phase_event_t;

/* 飞行阶段配置 */
typedef struct {
    float takeoff_throttle;      /* 起飞目标油门, 默认 0.55 */
    float hover_throttle;        /* 悬停油门, 默认 0.50 */
    float takeoff_ramp_rate;     /* 油门上升速率 (/s), 默认 0.3 */
    float landing_descent_rate;  /* 降落油门下降速率 (/s), 默认 0.15 */
    float landing_throttle_min;  /* 降落最低油门 (触地判定), 默认 0.05 */
    float hover_max_drift_deg;   /* 悬停最大允许漂移角度 (度), 默认 15 */
    float tilt_emergency_deg;    /* 紧急停机倾斜角 (度), 默认 60 */
} flight_phase_config_t;

#define FLIGHT_PHASE_CONFIG_DEFAULT { \
    .takeoff_throttle     = 0.55f,   \
    .hover_throttle       = 0.50f,   \
    .takeoff_ramp_rate    = 0.3f,    \
    .landing_descent_rate = 0.15f,   \
    .landing_throttle_min = 0.05f,   \
    .hover_max_drift_deg  = 15.0f,   \
    .tilt_emergency_deg   = 60.0f,   \
}

/* 飞行阶段状态机 (结构体) */
typedef struct {
    flight_phase_state_t phase;
    flight_phase_config_t cfg;

    /* 起飞/降落斜坡 */
    float current_throttle;      /* 当前油门目标 (斜坡中) */
    float target_throttle;       /* 最终目标油门 */
    bool  ramp_active;           /* 是否正在爬坡 */

    /* 悬停 */
    float hover_entry_roll;      /* 进入悬停时的 roll */
    float hover_entry_pitch;     /* 进入悬停时的 pitch */

    /* 着陆检测 */
    float landing_timer;         /* 降落持续时间 (秒) */

    /* 统计 */
    float phase_time;            /* 当前阶段已持续时间 (秒) */
    float total_flight_time;     /* 总飞行时间 (秒) */

    /* 上一次事件 */
    flight_phase_event_t last_event;
} flight_phase_t;

/* ---------- 生命周期 ---------- */

/**
 * @brief 初始化飞行阶段状态机
 */
void flight_phase_init(flight_phase_t *fp, const flight_phase_config_t *cfg);

/* ---------- 状态转换 ---------- */

/**
 * @brief 请求起飞 (IDLE → TAKEOFF)
 * @return 0=成功, -1=当前状态不允许
 */
int  flight_phase_request_takeoff(flight_phase_t *fp);

/**
 * @brief 请求降落 (HOVER/TAKEOFF → LANDING)
 * @return 0=成功, -1=当前状态不允许
 */
int  flight_phase_request_landing(flight_phase_t *fp);

/**
 * @brief 紧急停机 — 任意状态 → GROUNDED
 */
void flight_phase_emergency(flight_phase_t *fp);

/**
 * @brief 重置到 IDLE (着陆/上锁后)
 */
void flight_phase_reset(flight_phase_t *fp);

/* ---------- 主循环更新 ---------- */

/**
 * @brief 每个控制周期调用
 * @param fp        状态机
 * @param roll_deg  当前 roll (度)
 * @param pitch_deg 当前 pitch (度)
 * @param dt        周期 (秒)
 * @return          本周期输出的油门目标 (0~1)
 */
float flight_phase_update(flight_phase_t *fp,
                           float roll_deg,
                           float pitch_deg,
                           float dt);

/* ---------- 查询 ---------- */

flight_phase_state_t flight_phase_get(const flight_phase_t *fp);
const char *flight_phase_name(flight_phase_state_t phase);
flight_phase_event_t flight_phase_last_event(const flight_phase_t *fp);
bool flight_phase_in_flight(const flight_phase_t *fp);

#endif /* PACER_FLIGHT_PHASE_H */
