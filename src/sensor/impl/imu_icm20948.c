/**
 * @file imu_icm20948.c
 * @brief ICM20948 九轴 IMU 驱动实现
 *
 * 芯片内部架构:
 *   - 加速度计 + 陀螺仪 (主芯片)
 *   - AK09916 磁力计 (通过内部 I2C master 读取)
 *
 * 通信: I2C, 支持多 Bank 寄存器 (通过 BANK_SEL 切换)
 *
 * 关键流程:
 *   1. 复位 → 等待 → 设置时钟源
 *   2. Bank 2: 配置加速度计/陀螺仪 量程和采样率
 *   3. Bank 3: 配置内部 I2C master 读取磁力计
 *   4. Bank 0: 启用传感器, 读取数据
 */

#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "hal/hal_i2c.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

/* ================ 寄存器定义 ================ */

/* Bank 0 — 主控制 + 数据 */
#define R0_WHO_AM_I         0x00
#define R0_USER_CTRL        0x03
#define R0_PWR_MGMT_1       0x06
#define R0_PWR_MGMT_2       0x07
#define R0_INT_STATUS       0x13
#define R0_ACCEL_XOUT_H     0x2D
#define R0_GYRO_XOUT_H      0x33
#define R0_TEMP_OUT_H        0x39
#define R0_BANK_SEL         0x7F
#define ICM20948_ID         0xEA

/* Bank 2 — 传感器配置 */
#define R2_GYRO_SMPLRT_DIV  0x00
#define R2_GYRO_CONFIG_1    0x01
#define R2_ACCEL_SMPLRT_1   0x10
#define R2_ACCEL_SMPLRT_2   0x11
#define R2_ACCEL_CONFIG     0x14

/* Bank 3 — I2C master (读磁力计) */
#define R3_I2C_MST_CTRL     0x01
#define R3_SLV0_ADDR        0x03
#define R3_SLV0_REG         0x04
#define R3_SLV0_CTRL        0x05
#define R3_SLV0_DO          0x06
#define R3_SLV1_ADDR        0x07
#define R3_SLV1_REG         0x08
#define R3_SLV1_CTRL        0x09
#define R3_EXT_SENS_DATA    0x3B

/* AK09916 磁力计 */
#define AK_ADDR             0x0C
#define AK_ST1              0x10
#define AK_HXL              0x11
#define AK_CNTL2            0x31

/* ================ 量程灵敏度 ================ */

/* 加速度计 LSB/m/s² (量程 → 灵敏度) */
static float accel_scale_for_range(int range_g)
{
    /* ICM20948 16-bit ADC, 灵敏度 = range_g * 9.80665f / 32768 */
    return (float)range_g * 9.80665f / 32768.0f;
}

/* 陀螺仪 LSB/(°/s) */
static float gyro_scale_for_range(int range_dps)
{
    return (float)range_dps / 32768.0f;
}

/* ================ 模块内部状态 ================ */

static struct {
    i2c_dev_t i2c;
    float     accel_scale;    /* raw → m/s² */
    float     gyro_scale;     /* raw → °/s */
    vec3_t    gyro_bias;      /* 校准偏移 */
    bool      ready;
} self;

/* ================ 辅助函数 ================ */

/** 切换 Bank (0~3) */
static int select_bank(uint8_t bank)
{
    return i2c_write_reg(&self.i2c, R0_BANK_SEL, (bank << 4) & 0xF0);
}

/** 合并高低字节 */
static inline int16_t to_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/** 单调时间 (µs) */
static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000;
}

/* ================ 驱动实现 ================ */

static int icm_init(const imu_config_t *cfg)
{
    if (self.ready) return 0;

    memset(&self, 0, sizeof(self));

    /* 1. 打开 I2C */
    if (i2c_open(&self.i2c, cfg->i2c_bus, cfg->i2c_addr) != 0)
        return -1;

    /* 2. 软复位 */
    select_bank(0);
    i2c_write_reg(&self.i2c, R0_PWR_MGMT_1, 0x80);  /* DEVICE_RESET */
    usleep(50000);
    i2c_write_reg(&self.i2c, R0_PWR_MGMT_1, 0x01);  /* CLKSEL=PLL */
    usleep(50000);

    /* 3. 检查 ID */
    uint8_t id = 0;
    i2c_read_reg(&self.i2c, R0_WHO_AM_I, &id);
    if (id != ICM20948_ID) {
        fprintf(stderr, "[ICM20948] WHO_AM_I=0x%02x, expected 0x%02x\n", id, ICM20948_ID);
        i2c_close(&self.i2c);
        return -2;
    }
    printf("[ICM20948] detected, ID=0x%02x\n", id);

    /* 4. 配置传感器 — Bank 2 */
    select_bank(2);

    /* 加速度计采样率 */
    uint16_t a_div = (uint16_t)(1125.0f / cfg->sample_rate_hz) - 1;
    i2c_write_reg(&self.i2c, R2_ACCEL_SMPLRT_1, (a_div >> 8) & 0xFF);
    i2c_write_reg(&self.i2c, R2_ACCEL_SMPLRT_2, a_div & 0xFF);

    /* 加速度计量程: 0=±2g, 1=±4g, 2=±8g, 3=±16g */
    uint8_t a_fs = 0;
    if (cfg->accel_range_g <= 2) a_fs = 0;
    else if (cfg->accel_range_g <= 4) a_fs = 1;
    else if (cfg->accel_range_g <= 8) a_fs = 2;
    else a_fs = 3;
    i2c_write_reg(&self.i2c, R2_ACCEL_CONFIG, (a_fs << 1) | 0x01);
    self.accel_scale = accel_scale_for_range(cfg->accel_range_g);

    /* 陀螺仪采样率 */
    uint8_t g_div = (uint8_t)(1100.0f / cfg->sample_rate_hz) - 1;
    i2c_write_reg(&self.i2c, R2_GYRO_SMPLRT_DIV, g_div);

    /* 陀螺仪量程: 0=±250, 1=±500, 2=±1000, 3=±2000 */
    uint8_t g_fs = 0;
    if (cfg->gyro_range_dps <= 250) g_fs = 0;
    else if (cfg->gyro_range_dps <= 500) g_fs = 1;
    else if (cfg->gyro_range_dps <= 1000) g_fs = 2;
    else g_fs = 3;
    i2c_write_reg(&self.i2c, R2_GYRO_CONFIG_1, (g_fs << 1) | 0x01);
    self.gyro_scale = gyro_scale_for_range(cfg->gyro_range_dps);

    /* 5. 配置磁力计 — Bank 3 */
    select_bank(3);
    i2c_write_reg(&self.i2c, R3_I2C_MST_CTRL, 0x17);  /* 400kHz */

    /* SLV0: 读 AK09916 (ST1 + 8bytes = 9) */
    i2c_write_reg(&self.i2c, R3_SLV0_ADDR, AK_ADDR | 0x80);
    i2c_write_reg(&self.i2c, R3_SLV0_REG,  AK_ST1);
    i2c_write_reg(&self.i2c, R3_SLV0_CTRL, 0x89);

    /* SLV1: 写 AK09916 continuous mode */
    i2c_write_reg(&self.i2c, R3_SLV1_ADDR, AK_ADDR);
    i2c_write_reg(&self.i2c, R3_SLV1_REG,  AK_CNTL2);
    i2c_write_reg(&self.i2c, R3_SLV0_DO,   0x02);     /* 100Hz */
    i2c_write_reg(&self.i2c, R3_SLV1_CTRL, 0x81);

    /* 6. 启用 I2C master + 传感器 — Bank 0 */
    select_bank(0);
    i2c_write_reg(&self.i2c, R0_USER_CTRL, 0x20);     /* I2C_MST_EN */
    usleep(10000);
    i2c_write_reg(&self.i2c, R0_PWR_MGMT_2, 0x00);    /* 全部启用 */

    self.ready = true;
    printf("[ICM20948] accel=±%dg, gyro=±%ddps, rate=%dHz\n",
           cfg->accel_range_g, cfg->gyro_range_dps, cfg->sample_rate_hz);
    return 0;
}

static void icm_deinit(void)
{
    if (self.ready) {
        i2c_close(&self.i2c);
        self.ready = false;
        printf("[ICM20948] closed\n");
    }
}

static int icm_read(imu_sample_t *out)
{
    if (!self.ready) return -1;

    uint8_t raw[14];

    /* 加速度 (6) + 陀螺仪 (6) + 温度 (2) */
    select_bank(0);
    if (i2c_read_regs(&self.i2c, R0_ACCEL_XOUT_H, raw, 14) < 0)
        return -2;

    out->timestamp_us = now_us();

    out->accel.x = to_i16(raw[0], raw[1]) * self.accel_scale;
    out->accel.y = to_i16(raw[2], raw[3]) * self.accel_scale;
    out->accel.z = to_i16(raw[4], raw[5]) * self.accel_scale;

    out->gyro.x = to_i16(raw[6],  raw[7])  * self.gyro_scale - self.gyro_bias.x;
    out->gyro.y = to_i16(raw[8],  raw[9])  * self.gyro_scale - self.gyro_bias.y;
    out->gyro.z = to_i16(raw[10], raw[11]) * self.gyro_scale - self.gyro_bias.z;

    out->temperature = to_i16(raw[12], raw[13]) / 333.87f + 21.0f;

    /* 磁力计 (ext_sens_data, 9 bytes) */
    uint8_t mag[9];
    if (i2c_read_regs(&self.i2c, R3_EXT_SENS_DATA, mag, 9) >= 0) {
        select_bank(0);
        out->mag.x = (float)(int16_t)(mag[2] | (mag[3] << 8));
        out->mag.y = (float)(int16_t)(mag[4] | (mag[5] << 8));
        out->mag.z = (float)(int16_t)(mag[6] | (mag[7] << 8));
    }

    return 0;
}

static int icm_calibrate(int samples)
{
    if (!self.ready) return -1;

    printf("[ICM20948] calibrating gyro (%d samples)... keep still!\n", samples);

    vec3_t sum = {0};
    imu_sample_t sample;

    for (int i = 0; i < samples; i++) {
        if (icm_read(&sample) < 0) return -2;
        sum.x += sample.gyro.x;
        sum.y += sample.gyro.y;
        sum.z += sample.gyro.z;
        usleep(5000);
    }

    self.gyro_bias.x = sum.x / samples;
    self.gyro_bias.y = sum.y / samples;
    self.gyro_bias.z = sum.z / samples;

    printf("[ICM20948] bias = [%.3f, %.3f, %.3f] °/s\n",
           self.gyro_bias.x, self.gyro_bias.y, self.gyro_bias.z);
    return 0;
}

static void icm_set_bias(const vec3_t *bias)
{
    self.gyro_bias = *bias;
}

static void icm_get_bias(vec3_t *out)
{
    *out = self.gyro_bias;
}

static int icm_self_test(void)
{
    uint8_t id = 0;
    select_bank(0);
    i2c_read_reg(&self.i2c, R0_WHO_AM_I, &id);
    return (id == ICM20948_ID) ? 0 : -1;
}

/* ================ 注册 ================ */

void imu_icm20948_register(void)
{
    static const imu_driver_impl_t impl = {
        .init      = icm_init,
        .deinit    = icm_deinit,
        .read      = icm_read,
        .calibrate = icm_calibrate,
        .set_bias  = icm_set_bias,
        .get_bias  = icm_get_bias,
        .self_test = icm_self_test,
    };
    imu_register_driver(&impl);
}
