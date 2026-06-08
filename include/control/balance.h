/**
 * @file balance.h
 * @brief 平衡控制 — 三环串级 PID
 *
 * 控制架构:
 *   速度环 -> 角度环 -> 电机输出
 *   转向环 -> 左右差速
 */

#ifndef PACER_BALANCE_H
#define PACER_BALANCE_H

#include "control/pid.h"
#include "sensors/imu.h"
#include "sensors/attitude.h"
#include <stdbool.h>

/* 平衡控制器状态 */
typedef struct {
    /* 三个 PID 控制器 */
    pid_t angle_pid;    /* 平衡环 (角度 -> PWM) */
    pid_t speed_pid;    /* 速度环 (速度 -> 目标角度) */
    pid_t steer_pid;    /* 转向环 (偏航率 -> 差速) */

    /* 输出 */
    float motor_left;   /* 左电机 PWM [-1, 1] */
    float motor_right;  /* 右电机 PWM [-1, 1] */

    /* 状态 */
    bool  running;
    bool  emergency;
    float target_speed;  /* 目标速度 (m/s) */
    float target_steer;  /* 目标转向 (rad/s) */

    /* 调试用 */
    float debug_angle;
    float debug_angle_target;
    float debug_speed_error;
} balance_t;

/**
 * @brief 初始化平衡控制器
 */
void balance_init(balance_t *bal);

/**
 * @brief 平衡控制更新 (每控制周期调用)
 * @param bal   控制器
 * @param imu   IMU 数据
 * @param att   姿态角
 * @param dt    时间间隔 (秒)
 */
void balance_update(balance_t *bal, const imu_data_t *imu,
                    const attitude_t *att, float dt);

/**
 * @brief 设置目标速度
 */
void balance_set_speed(balance_t *bal, float speed);

/**
 * @brief 设置目标转向
 */
void balance_set_steer(balance_t *bal, float steer);

/**
 * @brief 启停
 */
void balance_start(balance_t *bal);
void balance_stop(balance_t *bal);

/**
 * @brief 紧急停机
 */
void balance_emergency(balance_t *bal);

#endif /* PACER_BALANCE_H */
