/**
 * @file imu.h
 * @brief ICM20948 九轴 IMU 驱动
 */

#ifndef PACER_IMU_H
#define PACER_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* ICM20948 数据结构 */
typedef struct {
    float accel[3];     // 加速度 (m/s²): X, Y, Z
    float gyro[3];      // 角速度 (deg/s): X, Y, Z
    float mag[3];       // 磁力计 (µT): X, Y, Z
    float temp;         // 温度 (°C)
    uint64_t timestamp; // 时间戳 (us)
} imu_data_t;

/* IMU 初始化配置 */
typedef struct {
    int i2c_bus;
    uint8_t i2c_addr;
    int sample_rate;        // Hz
    int accel_range;        // ±2/4/8/16g
    int gyro_range;         // ±250/500/1000/2000 dps
} imu_config_t;

/* 默认配置 */
#define IMU_DEFAULT_CONFIG { \
    .i2c_bus = 1,            \
    .i2c_addr = 0x68,       \
    .sample_rate = 200,      \
    .accel_range = 4,        \
    .gyro_range = 1000       \
}

/**
 * @brief 初始化 ICM20948
 * @param cfg IMU 配置
 * @return 0=成功, <0=错误码
 */
int imu_init(const imu_config_t *cfg);

/**
 * @brief 读取 IMU 数据 (非阻塞)
 * @param data 输出数据
 * @return 0=成功, <0=错误码
 */
int imu_read(imu_data_t *data);

/**
 * @brief 软复位 IMU
 */
int imu_reset(void);

/**
 * @brief 自检
 * @return 0=通过
 */
int imu_self_test(void);

/**
 * @brief 关闭 IMU
 */
void imu_close(void);

/**
 * @brief 校准 (静止状态调用)
 * @param samples 采样数量
 * @return 0=成功
 */
int imu_calibrate(int samples);

/**
 * @brief 获取陀螺仪偏移
 * @param offset 输出偏移 [x, y, z]
 */
void imu_get_gyro_offset(float offset[3]);

/**
 * @brief 设置陀螺仪偏移
 * @param offset 偏移 [x, y, z]
 */
void imu_set_gyro_offset(const float offset[3]);

#endif /* PACER_IMU_H */
