/**
 * @file balance.h
 * @brief 平衡控制器 — 三环串级 PID
 *
 * 控制架构:
 *
 *   目标速度 ──→ [速度环] ──→ 目标倾斜角
 *                                    │
 *                              [角度环] ──→ 基础 PWM
 *                                    │
 *   目标转向 ──→ [转向环] ──→ 差速 PWM
 *                                    │
 *                          ┌─────────┴─────────┐
 *                          ↓                   ↓
 *                       左电机              右电机
 *
 * 三环之间的关系:
 *   - 速度环 外环: 控制小车前后运动
 *   - 角度环 中环: 保持车体直立
 *   - 转向环 辅助: 控制左右方向
 */

#ifndef PACER_BALANCE_H
#define PACER_BALANCE_H

#include "ctrl/pid.h"
#include "filter/filter.h"
#include "sensor/imu.h"

/* 控制器输出 */
typedef struct {
    float motor_left;    /* 左电机 PWM (-range ~ +range) */
    float motor_right;   /* 右电机 PWM (-range ~ +range) */
    bool  emergency;     /* 是否触发紧急停机 */
} balance_output_t;

/* 调试信息 (可选, 串口打印用) */
typedef struct {
    float roll;             /* 当前横滚角 */
    float roll_target;      /* 目标横滚角 */
    float gyro_x;           /* 陀螺仪 X 轴 */
    float speed_integral;   /* 速度环积分项 */
} balance_debug_t;

/* 平衡控制器 */
typedef struct {
    pid_t pid_angle;        /* 角度环 */
    pid_t pid_speed;        /* 速度环 */
    pid_t pid_steer;        /* 转向环 */

    float target_speed;     /* 目标前进速度 */
    float target_yaw_rate;  /* 目标偏航角速度 */

    balance_output_t output;
    balance_debug_t  debug;

    bool enabled;
} balance_t;

/* ---------- 生命周期 ---------- */

/**
 * @brief 用默认参数初始化
 */
void balance_init(balance_t *b);

/* ---------- 控制 ---------- */

/**
 * @brief 一个控制周期的计算
 * @param b       控制器
 * @param att     当前姿态
 * @param sample  IMU 原始数据
 * @param dt      周期 (秒)
 * @return        电机输出
 */
const balance_output_t *balance_update(balance_t *b,
                                       const attitude_t *att,
                                       const imu_sample_t *sample,
                                       float dt);

/* ---------- 设定 ---------- */

void balance_set_speed(balance_t *b, float speed);
void balance_set_steer(balance_t *b, float yaw_rate);
void balance_enable(balance_t *b, bool on);

/* ---------- 运行时调参 ---------- */

void balance_set_angle_gains(float kp, float ki, float kd);
void balance_set_speed_gains(float kp, float ki, float kd);
void balance_set_angle_offset(float offset_deg);

/**
 * @brief 获取调试数据
 */
const balance_debug_t *balance_get_debug(const balance_t *b);

#endif /* PACER_BALANCE_H */
