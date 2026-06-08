/**
 * @file imu.h
 * @brief IMU 传感器统一接口
 *
 * 上层只依赖这个接口，不关心底层是 ICM20948 还是 MPU6050。
 */

#ifndef PACER_IMU_H
#define PACER_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* 三轴浮点向量 */
typedef struct {
    float x, y, z;
} vec3_t;

/* IMU 采样数据 */
typedef struct {
    vec3_t   accel;         /* 加速度 (m/s²) */
    vec3_t   gyro;          /* 角速度 (°/s)  */
    vec3_t   mag;           /* 磁力 (µT, 可选) */
    float    temperature;   /* 温度 (°C) */
    uint64_t timestamp_us;  /* 单调时间戳 (µs) */
} imu_sample_t;

/* IMU 配置 */
typedef struct {
    int  i2c_bus;            /* I2C 总线号 */
    uint8_t i2c_addr;        /* I2C 地址 */
    int  sample_rate_hz;     /* 采样率 */
    int  accel_range_g;      /* 加速度量程 (±g) */
    int  gyro_range_dps;     /* 陀螺仪量程 (±°/s) */
} imu_config_t;

#define IMU_CONFIG_DEFAULT { \
    .i2c_bus       = 1,      \
    .i2c_addr      = 0x68,   \
    .sample_rate_hz = 200,    \
    .accel_range_g  = 4,      \
    .gyro_range_dps = 1000    \
}

/* ---------- 生命周期 ---------- */

int  imu_init(const imu_config_t *cfg);
void imu_deinit(void);

/* ---------- 数据 ---------- */

/**
 * @brief 读取一帧数据 (非阻塞)
 * @return 0=成功, -1=未初始化, -2=通信失败
 */
int  imu_read(imu_sample_t *out);

/* ---------- 校准 ---------- */

/**
 * @brief 陀螺仪零偏校准 (保持静止)
 * @param samples  采样数量, 推荐 500
 * @return 0=成功
 */
int  imu_calibrate_gyro(int samples);

void imu_set_gyro_bias(const vec3_t *bias);
void imu_get_gyro_bias(vec3_t *out);

/* ---------- 诊断 ---------- */

/**
 * @brief 自检 (检查 WHO_AM_I)
 */
int  imu_self_test(void);

/* ---------- 驱动注册 (内部使用) ---------- */

typedef struct {
    int  (*init)(const imu_config_t *cfg);
    void (*deinit)(void);
    int  (*read)(imu_sample_t *out);
    int  (*calibrate)(int samples);
    void (*set_bias)(const vec3_t *bias);
    void (*get_bias)(vec3_t *out);
    int  (*self_test)(void);
} imu_driver_impl_t;

void imu_register_driver(const imu_driver_impl_t *impl);

#endif /* PACER_IMU_H */
