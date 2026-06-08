/**
 * @file imu.c
 * @brief IMU 接口层 — 策略分发
 *
 * 上层调用 imu_init / imu_read 等函数，
 * 底层由具体传感器 (ICM20948/MPU6050/...) 实现。
 * 通过函数指针实现"多态"。
 */

#include "sensor/imu.h"
#include <stddef.h>
#include <string.h>

/* 当前注册的驱动 */
static imu_driver_impl_t g_driver = {0};

/* ---------- 注册 ---------- */

void imu_register_driver(const imu_driver_impl_t *drv)
{
    memcpy(&g_driver, drv, sizeof(imu_driver_impl_t));
}

/* ---------- 接口实现 ---------- */

int imu_init(const imu_config_t *cfg)
{
    if (!g_driver.init) return -1;
    return g_driver.init(cfg);
}

void imu_deinit(void)
{
    if (g_driver.deinit) g_driver.deinit();
}

int imu_read(imu_sample_t *out)
{
    if (!g_driver.read) return -1;
    return g_driver.read(out);
}

int imu_calibrate_gyro(int samples)
{
    if (!g_driver.calibrate) return -1;
    return g_driver.calibrate(samples);
}

void imu_set_gyro_bias(const vec3_t *bias)
{
    if (g_driver.set_bias) g_driver.set_bias(bias);
}

void imu_get_gyro_bias(vec3_t *out)
{
    if (g_driver.get_bias) g_driver.get_bias(out);
}

int imu_self_test(void)
{
    if (!g_driver.self_test) return -1;
    return g_driver.self_test();
}
