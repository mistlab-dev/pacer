/**
 * @file attitude.c
 * @brief 姿态解算实现
 *
 * 支持 Madgwick 梯度下降和一阶互补滤波两种算法。
 * 平衡车主要用 roll 角（横滚）来控制前后平衡。
 */

#include "sensors/attitude.h"
#include "config.h"
#include <math.h>
#include <string.h>

#define DEG2RAD(x)  ((x) * (float)M_PI / 180.0f)
#define RAD2DEG(x)  ((x) * 180.0f / (float)M_PI)

static struct {
    quaternion_t q;
    attitude_t current;
    float beta;     /* Madgwick */
    float alpha;    /* 互补滤波 */
    bool initialized;
} g_att;

/* 快速平方根倒数 (Quake III) */
static inline float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void attitude_init(void)
{
    g_att.q.q[0] = 1.0f;   /* w */
    g_att.q.q[1] = 0.0f;   /* x */
    g_att.q.q[2] = 0.0f;   /* y */
    g_att.q.q[3] = 0.0f;   /* z */
    g_att.beta = MADGWICK_BETA;
    g_att.alpha = COMPLEMENTARY_ALPHA;
    g_att.current.roll = 0;
    g_att.current.pitch = 0;
    g_att.current.yaw = 0;
    g_att.initialized = true;
    printf("[ATT] Attitude initialized (Madgwick beta=%.3f, alpha=%.3f)\n",
           g_att.beta, g_att.alpha);
}

/**
 * Madgwick MARG 梯度下降滤波器
 * 参考: "An efficient orientation filter for IMU and MARG sensor arrays"
 */
attitude_t attitude_update_madgwick(const imu_data_t *imu, float dt)
{
    if (!g_att.initialized) attitude_init();

    float *q = g_att.q.q;
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float gx = DEG2RAD(imu->gyro[0]);
    float gy = DEG2RAD(imu->gyro[1]);
    float gz = DEG2RAD(imu->gyro[2]);
    float ax = imu->accel[0];
    float ay = imu->accel[1];
    float az = imu->accel[2];

    /* 归一化加速度 */
    float norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* 梯度下降修正 */
    float _2qw = 2.0f * w;
    float _2qx = 2.0f * x;
    float _2qy = 2.0f * y;
    float _2qz = 2.0f * z;

    /* 重力方向误差 */
    float hx = ay * _2qz - az * _2qy;
    float hy = az * _2qx - ax * _2qz;
    float hz = ax * _2qy - ay * _2qx;

    float f1 = 2.0f * (x * z - w * y) - ax;
    float f2 = 2.0f * (w * x + y * z) - ay;
    float f3 = 1.0f - 2.0f * (x * x + y * y) - az;

    /* 归一化梯度 */
    norm = inv_sqrt(f1 * f1 + f2 * f2 + f3 * f3);
    f1 *= norm;
    f2 *= norm;
    f3 *= norm;

    /* 四元数微分方程 */
    float beta_dt = g_att.beta * dt;
    w += 0.5f * (-_2qx * gy - _2qy * gz - _2qz * (-gz)) - beta_dt * hx;
    x += 0.5f * (_2qw * gy + _2qy * (-gx) + _2qz * (-gy)) - beta_dt * hy;
    y += 0.5f * (_2qw * (-gz) + _2qx * (-gx) + _2qz * gy) - beta_dt * hz;
    z += 0.5f * (_2qw * gx + _2qx * gy + _2qy * (-gx));

    /* 归一化四元数 */
    norm = inv_sqrt(w * w + x * x + y * y + z * z);
    q[0] = w * norm;
    q[1] = x * norm;
    q[2] = y * norm;
    q[3] = z * norm;

    /* 四元数 -> 欧拉角 */
    float sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    g_att.current.roll  = RAD2DEG(atan2f(sinr, cosr));

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    sinp = (sinp > 1.0f) ? 1.0f : ((sinp < -1.0f) ? -1.0f : sinp);
    g_att.current.pitch = RAD2DEG(asinf(sinp));

    float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    g_att.current.yaw   = RAD2DEG(atan2f(siny, cosy));

    return g_att.current;
}

/**
 * 互补滤波: 加速度计低频 + 陀螺仪高频
 * 简单但有效，平衡车够用
 */
attitude_t attitude_update_complementary(const imu_data_t *imu, float dt)
{
    if (!g_att.initialized) attitude_init();

    /* 加速度计求角度 (低频稳定，高频噪声大) */
    float accel_roll  = RAD2DEG(atan2f(imu->accel[1], imu->accel[2]));
    float accel_pitch = RAD2DEG(atan2f(-imu->accel[0],
                          sqrtf(imu->accel[1] * imu->accel[1] +
                                imu->accel[2] * imu->accel[2])));

    /* 陀螺仪积分 (高频准确，低频漂移) */
    float alpha = g_att.alpha;
    g_att.current.roll  = alpha * (g_att.current.roll  + imu->gyro[0] * dt)
                         + (1.0f - alpha) * accel_roll;
    g_att.current.pitch = alpha * (g_att.current.pitch + imu->gyro[1] * dt)
                         + (1.0f - alpha) * accel_pitch;
    g_att.current.yaw  += imu->gyro[2] * dt; /* 纯积分，会漂移 */

    return g_att.current;
}

attitude_t attitude_get_current(void)
{
    return g_att.current;
}

void attitude_reset(void)
{
    g_att.q.q[0] = 1.0f;
    g_att.q.q[1] = 0.0f;
    g_att.q.q[2] = 0.0f;
    g_att.q.q[3] = 0.0f;
    g_att.current.roll = 0;
    g_att.current.pitch = 0;
    g_att.current.yaw = 0;
}

void attitude_set_beta(float beta)
{
    g_att.beta = beta;
}

void attitude_set_alpha(float alpha)
{
    g_att.alpha = alpha;
}
