/**
 * @file filter.h
 * @brief 姿态滤波器统一接口
 *
 * 上层只依赖这个接口，不关心底层是 Madgwick 还是互补滤波。
 */

#ifndef PACER_FILTER_H
#define PACER_FILTER_H

#include "sensor/imu.h"

/* 姿态角 (度) */
typedef struct {
    float roll;      /* 横滚 — 平衡车核心控制量 */
    float pitch;     /* 俯仰 */
    float yaw;       /* 航向 */
} attitude_t;

/* 滤波器类型 */
typedef enum {
    FILTER_COMPLEMENTARY = 0,
    FILTER_MADGWICK      = 1,
} filter_type_t;

/* 滤波器参数 */
typedef struct {
    filter_type_t type;
    float alpha;     /* 互补滤波系数 (0~1) */
    float beta;      /* Madgwick 步长 */
} filter_config_t;

#define FILTER_CONFIG_DEFAULT { \
    .type  = FILTER_MADGWICK,   \
    .alpha = 0.98f,             \
    .beta  = 0.04f              \
}

/* ---------- 生命周期 ---------- */

int  filter_init(const filter_config_t *cfg);
void filter_reset(void);

/* ---------- 更新 ---------- */

/**
 * @brief 输入 IMU 数据，输出姿态角
 * @param sample IMU 采样
 * @param dt      时间间隔 (秒)
 * @return 姿态 (度)
 */
attitude_t filter_update(const imu_sample_t *sample, float dt);

/**
 * @brief 获取当前姿态 (不更新)
 */
attitude_t filter_get_attitude(void);

#endif /* PACER_FILTER_H */
