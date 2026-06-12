/**
 * @file filter.c
 * @brief 姿态滤波器 — 接口层
 *
 * 根据 config 选择 Madgwick 或互补滤波。
 */

#include "filter/filter.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define DEG2RAD(x)  ((x) * (float)M_PI / 180.0f)
#define RAD2DEG(x)  ((x) * 180.0f / (float)M_PI)

/* ================ 内部状态 ================ */

static struct {
    filter_config_t cfg;
    attitude_t      att;

    /* Madgwick 四元数 q = {w, x, y, z} */
    float q[4];

    /* 互补滤波上一帧角度 */
    float comp_roll;
    float comp_pitch;
    float comp_yaw;
} g;

/* ================ Madgwick ================ */

/**
 * Madgwick 梯度下降姿态估计
 *
 * 参考: Sebastian Madgwick, "An efficient orientation filter
 *       for inertial and inertial/magnetic sensor arrays", 2010
 *
 * 核心思想: 用加速度计方向修正陀螺仪积分漂移
 * beta 控制修正强度: 越大越信任加速度计, 越小越信任陀螺仪
 */
static attitude_t madgwick_update(const imu_sample_t *s, float dt)
{
    float *q = g.q;
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float gx = DEG2RAD(s->gyro.x);
    float gy = DEG2RAD(s->gyro.y);
    float gz = DEG2RAD(s->gyro.z);

    float ax = s->accel.x, ay = s->accel.y, az = s->accel.z;

    /* 归一化加速度 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) return g.att;
    float inv_n = 1.0f / norm;
    ax *= inv_n; ay *= inv_n; az *= inv_n;

    /* 重力方向在体坐标系下的估计 (从四元数推算) */
    float hx = 2.0f * (x * z - w * y);
    float hy = 2.0f * (w * x + y * z);
    float hz = w * w - x * x - y * y + z * z;

    /* 误差 = 估计方向 × 实测加速度方向 */
    float ex = ay * hz - az * hy;
    float ey = az * hx - ax * hz;
    float ez = ax * hy - ay * hx;

    /* 误差归一化 */
    float e_norm = sqrtf(ex * ex + ey * ey + ez * ez);
    if (e_norm > 0.001f) {
        float inv_e = 1.0f / e_norm;
        ex *= inv_e; ey *= inv_e; ez *= inv_e;
    }

    /* 用误差修正陀螺仪 (梯度下降步) */
    float beta = g.cfg.beta;
    gx += beta * ex;
    gy += beta * ey;
    gz += beta * ez;

    /* 四元数一阶积分 */
    float half_dt = 0.5f * dt;
    w += (-x * gx - y * gy - z * gz) * half_dt;
    x += ( w * gx + y * gz - z * gy) * half_dt;
    y += ( w * gy - x * gz + z * gx) * half_dt;
    z += ( w * gz + x * gy - y * gx) * half_dt;

    /* 归一化四元数 */
    norm = sqrtf(w * w + x * x + y * y + z * z);
    float inv_q = 1.0f / norm;
    q[0] = w * inv_q;
    q[1] = x * inv_q;
    q[2] = y * inv_q;
    q[3] = z * inv_q;

    /* 四元数 → 欧拉角 */
    float sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    g.att.roll = RAD2DEG(atan2f(sinr, cosr));

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    g.att.pitch = RAD2DEG(asinf(sinp));

    float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    g.att.yaw = RAD2DEG(atan2f(siny, cosy));

    return g.att;
}

/* ================ 互补滤波 ================ */

/**
 * 一阶互补滤波
 *
 * 原理: 陀螺仪积分 (高频好, 低频漂移)
 *       + 加速度计角度 (低频好, 高频噪声)
 *       → 高通取陀螺仪 + 低通取加速度计
 *
 * alpha 越大越信任陀螺仪 (典型 0.96~0.99)
 */
static attitude_t complementary_update(const imu_sample_t *s, float dt)
{
    /* 加速度计求角度 */
    float acc_roll  = RAD2DEG(atan2f(s->accel.y, s->accel.z));
    float acc_pitch = RAD2DEG(atan2f(-s->accel.x,
                           sqrtf(s->accel.y * s->accel.y + s->accel.z * s->accel.z)));

    float a = g.cfg.alpha;

    /* 融合 */
    g.comp_roll  = a * (g.comp_roll  + s->gyro.x * dt) + (1.0f - a) * acc_roll;
    g.comp_pitch = a * (g.comp_pitch + s->gyro.y * dt) + (1.0f - a) * acc_pitch;
    g.comp_yaw  += s->gyro.z * dt;  /* 无修正, 会漂移 */

    g.att.roll  = g.comp_roll;
    g.att.pitch = g.comp_pitch;
    g.att.yaw   = g.comp_yaw;

    return g.att;
}

/* ================ 公开接口 ================ */

int filter_init(const filter_config_t *cfg)
{
    g.cfg = *cfg;

    /* Madgwick 初始四元数 */
    g.q[0] = 1.0f;
    g.q[1] = g.q[2] = g.q[3] = 0.0f;

    g.comp_roll = g.comp_pitch = g.comp_yaw = 0.0f;

    const char *name = (cfg->type == FILTER_MADGWICK) ? "Madgwick" : "Complementary";
    printf("[FILTER] %s init (alpha=%.2f, beta=%.3f)\n",
           name, cfg->alpha, cfg->beta);
    return 0;
}

void filter_reset(void)
{
    g.q[0] = 1.0f;
    g.q[1] = g.q[2] = g.q[3] = 0.0f;
    g.comp_roll = g.comp_pitch = g.comp_yaw = 0.0f;
    g.att.roll = g.att.pitch = g.att.yaw = 0.0f;
}

attitude_t filter_update(const imu_sample_t *sample, float dt)
{
    if (g.cfg.type == FILTER_MADGWICK)
        return madgwick_update(sample, dt);
    else
        return complementary_update(sample, dt);
}

attitude_t filter_get_attitude(void)
{
    return g.att;
}
