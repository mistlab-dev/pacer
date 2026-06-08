/**
 * @file attitude.h
 * @brief 姿态解算 (Madgwick / 互补滤波)
 */

#ifndef PACER_ATTITUDE_H
#define PACER_ATTITUDE_H

#include "sensors/imu.h"
#include <stdbool.h>

/* 姿态角 (度) */
typedef struct {
    float roll;     // 横滚 (左右倾斜) — 平衡车核心
    float pitch;    // 俯仰 (前后倾斜)
    float yaw;      // 航向
} attitude_t;

/* 四元数 (Madgwick 内部使用) */
typedef struct {
    float q[4];     // w, x, y, z
} quaternion_t;

/**
 * @brief 初始化姿态解算
 */
void attitude_init(void);

/**
 * @brief 更新姿态 (Madgwick)
 * @param imu IMU 原始数据
 * @param dt  时间间隔 (秒)
 * @return 姿态角 (度)
 */
attitude_t attitude_update_madgwick(const imu_data_t *imu, float dt);

/**
 * @brief 更新姿态 (互补滤波)
 * @param imu IMU 原始数据
 * @param dt  时间间隔 (秒)
 * @return 姿态角 (度)
 */
attitude_t attitude_update_complementary(const imu_data_t *imu, float dt);

/**
 * @brief 获取当前姿态
 */
attitude_t attitude_get_current(void);

/**
 * @brief 重置姿态
 */
void attitude_reset(void);

/**
 * @brief 设置 Madgwick beta 参数
 */
void attitude_set_beta(float beta);

/**
 * @brief 设置互补滤波 alpha 参数
 */
void attitude_set_alpha(float alpha);

#endif /* PACER_ATTITUDE_H */
