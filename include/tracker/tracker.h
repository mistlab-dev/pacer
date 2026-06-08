/**
 * @file tracker.h
 * @brief 目标追踪 — 融合蓝牙定位 + 摄像头检测
 *
 * 数据融合策略:
 *
 *   蓝牙信标 → 身份确认 + 粗方位 + 距离
 *   摄像头   → 精确角度 + 目标大小
 *
 *   ┌──────────┐     ┌──────────┐
 *   │  beacon  │     │  camera  │
 *   │ (方位+距离)│    │ (精确角度) │
 *   └─────┬────┘     └─────┬────┘
 *         │                │
 *         └───────┬────────┘
 *                 ↓
 *          ┌──────────┐
 *          │  tracker │ ← 融合决策
 *          │ (输出指令) │
 *          └─────┬────┘
 *                ↓
 *         target_speed
 *         target_yaw_rate
 *         → 喂给 balance 控制器
 *
 * 追踪状态机:
 *
 *   IDLE → SCANNING → LOCKED → TRACKING → LOST → SCANNING
 *                     ↑                      │
 *                     └──────────────────────┘
 */

#ifndef PACER_TRACKER_H
#define PACER_TRACKER_H

#include "tracker/beacon.h"
#include "tracker/camera.h"
#include <stdbool.h>

/* 追踪状态 */
typedef enum {
    TRACK_IDLE,         /* 空闲, 不追踪 */
    TRACK_SCANNING,     /* 扫描中, 寻找目标 */
    TRACK_LOCKED,       /* 锁定目标, 准备追踪 */
    TRACK_TRACKING,     /* 追踪中 */
    TRACK_LOST,         /* 丢失目标, 原地旋转寻找 */
} tracker_state_t;

/* 追踪输出 — 喂给 balance 控制器 */
typedef struct {
    float target_speed;     /* 目标前进速度 (m/s) */
    float target_yaw_rate;  /* 目标转向角速度 (rad/s) */
    bool  should_stop;      /* 是否应该停止 */
} tracker_output_t;

/* 追踪配置 */
typedef struct {
    /* 蓝牙 */
    char  beacon_uuid[37];         /* 目标信标 UUID */
    int   hci_devs[BEACON_RX_COUNT]; /* 3个蓝牙适配器 hci 编号 */

    /* 摄像头 */
    camera_config_t camera;

    /* 追踪参数 */
    float follow_distance;     /* 跟随距离 (米), 默认 1.5 */
    float follow_speed_max;    /* 最大跟随速度 (m/s), 默认 0.5 */
    float turn_speed_max;      /* 最大转向速度 (rad/s), 默认 1.0 */
    float lost_timeout;        /* 丢失多久进入 LOST (秒), 默认 3.0 */
    float scan_rotate_speed;   /* 扫描时旋转速度 (rad/s), 默认 0.5 */
} tracker_config_t;

#define TRACKER_CONFIG_DEFAULT { \
    .beacon_uuid       = "AA BB CC DD EE FF 00 11 22 33 44 55 66 77 88 99", \
    .hci_devs          = {0, 1, 2}, \
    .camera            = CAMERA_CONFIG_DEFAULT, \
    .follow_distance   = 1.5f, \
    .follow_speed_max  = 0.5f, \
    .turn_speed_max    = 1.0f, \
    .lost_timeout      = 3.0f, \
    .scan_rotate_speed = 0.5f  \
}

/* ---------- 生命周期 ---------- */

int  tracker_init(const tracker_config_t *cfg);
void tracker_deinit(void);

/* ---------- 控制 ---------- */

/**
 * @brief 启动追踪
 */
void tracker_start(void);

/**
 * @brief 停止追踪
 */
void tracker_stop(void);

/**
 * @brief 运行一次更新 (主循环中调用)
 * @return 追踪输出, 喂给 balance_set_speed / balance_set_steer
 */
tracker_output_t tracker_update(void);

/* ---------- 状态 ---------- */

tracker_state_t tracker_get_state(void);

/**
 * @brief 获取调试信息
 */
const char *tracker_state_str(tracker_state_t s);

#endif /* PACER_TRACKER_H */
