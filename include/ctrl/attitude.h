/**
 * @file attitude.h
 * @brief 四旋翼姿态控制器 — 串级 PID
 *
 * 控制架构:
 *
 *   遥控输入
 *     │
 *     ├── throttle ─────────────────────────→ 混控器
 *     │
 *     ├── 目标 roll ──→ [角度环] ──→ [角速度环] ──→ 混控器 roll
 *     │
 *     ├── 目标 pitch ──→ [角度环] ──→ [角速度环] ──→ 混控器 pitch
 *     │
 *     └── 目标 yaw_rate ──────────→ [角速度环] ──→ 混控器 yaw
 *
 * 内环 (角速度环): 快速响应陀螺仪，稳定旋转
 * 外环 (角度环): 跟踪目标倾斜角，输出目标角速度
 *
 * 默认用角速度模式 (rate mode / acro):
 *   遥控杆直接控制角速度，不靠角度环
 *   如果需要自稳 (angle mode / self-level):
 *   使能 CFG_QUAD_ANGLE_MODE，遥控杆控制目标角度
 */

#ifndef PACER_ATTITUDE_H
#define PACER_ATTITUDE_H

#include "ctrl/pid.h"
#include "filter/filter.h"
#include "sensor/imu.h"

/* 控制器输出 */
typedef struct {
    float throttle;   /* 0~1 */
    float roll;       /* -1~+1 混控器输入 */
    float pitch;      /* -1~+1 */
    float yaw;        /* -1~+1 */
} attitude_output_t;

/* 调试信息 */
typedef struct {
    float roll;            /* 当前 roll (度) */
    float pitch;           /* 当前 pitch (度) */
    float roll_rate;       /* 当前 roll 角速度 (度/s) */
    float pitch_rate;      /* 当前 pitch 角速度 (度/s) */
    float yaw_rate;        /* 当前 yaw 角速度 (度/s) */
    float roll_target;     /* 目标 (度 或 度/s, 取决模式) */
    float pitch_target;
    float yaw_target;
} attitude_debug_t;

/* 控制模式 */
typedef enum {
    ATT_MODE_RATE,     /* 角速度模式 (acro) — 默认 */
    ATT_MODE_ANGLE,    /* 自稳模式 — 遥控杆控角度 */
} att_mode_t;

/* 姿态控制器 */
typedef struct {
    /* PID: 角速度环 (内环, 必需) */
    pacer_pid_t pid_roll_rate;
    pacer_pid_t pid_pitch_rate;
    pacer_pid_t pid_yaw_rate;

    /* PID: 角度环 (外环, 自稳模式用) */
    pacer_pid_t pid_roll_angle;
    pacer_pid_t pid_pitch_angle;

    /* 目标 */
    float target_throttle;
    float target_roll;       /* 角度模式: 度 | 角速度模式: 度/s */
    float target_pitch;
    float target_yaw_rate;   /* 度/s */

    att_mode_t mode;
    attitude_output_t output;
    attitude_debug_t  debug;
    bool enabled;
} attitude_ctrl_t;

/* ---------- 生命周期 ---------- */

/**
 * @brief 初始化姿态控制器 (默认参数)
 */
void attitude_init(attitude_ctrl_t *ac);

/* ---------- 控制 ---------- */

/**
 * @brief 一个控制周期的计算
 * @param ac      控制器
 * @param att     当前姿态 (from filter)
 * @param sample  IMU 原始数据 (for gyro rates)
 * @param dt      周期 (秒)
 * @return        控制输出
 */
const attitude_output_t *attitude_update(attitude_ctrl_t *ac,
                                          const attitude_t *att,
                                          const imu_sample_t *sample,
                                          float dt);

/* ---------- 设定 ---------- */

/**
 * @brief 设置遥控输入
 * @param ac          控制器
 * @param throttle    0~1 油门
 * @param roll_stick  -1~+1 遥控杆 (映射为角度或角速度)
 * @param pitch_stick -1~+1
 * @param yaw_stick   -1~+1
 */
void attitude_set_input(attitude_ctrl_t *ac,
                         float throttle,
                         float roll_stick,
                         float pitch_stick,
                         float yaw_stick);

/**
 * @brief 切换控制模式
 */
void attitude_set_mode(attitude_ctrl_t *ac, att_mode_t mode);

/**
 * @brief 使能/禁用
 */
void attitude_enable(attitude_ctrl_t *ac, bool on);

/* ---------- 运行时调参 ---------- */

void attitude_set_rate_gains(float roll_kp, float roll_ki, float roll_kd,
                              float pitch_kp, float pitch_ki, float pitch_kd,
                              float yaw_kp, float yaw_ki, float yaw_kd);

/**
 * @brief 获取调试数据
 */
const attitude_debug_t *attitude_get_debug(const attitude_ctrl_t *ac);

#endif /* PACER_ATTITUDE_H */
