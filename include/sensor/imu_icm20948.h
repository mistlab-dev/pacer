/**
 * @file imu_icm20948.h
 * @brief ICM20948 具体实现注册
 *
 * 调用 imu_icm20948_register() 将 ICM20948 驱动
 * 注册到 imu 接口层。这是"策略"和"实现"的连接点。
 */

#ifndef PACER_IMU_ICM20948_H
#define PACER_IMU_ICM20948_H

#include "sensor/imu.h"

/**
 * @brief 将 ICM20948 注册为 IMU 后端
 * 在 imu_init() 之前调用。
 */
void imu_icm20948_register(void);

#endif /* PACER_IMU_ICM20948_H */
