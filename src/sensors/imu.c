/**
 * @file imu.c
 * @brief ICM20948 九轴 IMU 驱动实现 (树莓派 I2C)
 *
 * ICM20948 = 加速度计 + 陀螺仪 + AKM 磁力计
 * 通过 I2C 通信，使用 pigpio 库
 */

#include "sensors/imu.h"
#include "config.h"
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

/* ============== ICM20948 寄存器 ============== */

/* Bank 0 */
#define REG_WHO_AM_I            0x00
#define REG_USER_CTRL           0x03
#define REG_LP_CONFIG           0x05
#define REG_PWR_MGMT_1          0x06
#define REG_PWR_MGMT_2          0x07
#define REG_INT_PIN_CFG         0x0F
#define REG_INT_ENABLE          0x10
#define REG_INT_STATUS          0x13
#define REG_ACCEL_XOUT_H        0x2D
#define REG_ACCEL_XOUT_L        0x2E
#define REG_GYRO_XOUT_H         0x33
#define REG_GYRO_XOUT_L         0x34
#define REG_TEMP_OUT_H           0x39
#define REG_TEMP_OUT_L           0x3A
#define REG_BANK_SEL            0x7F

/* 期望的 WHO_AM_I 值 */
#define ICM20948_WHO_AM_I       0xEA

/* Bank 2 */
#define REG2_GYRO_SMPLRT_DIV    0x00
#define REG2_GYRO_CONFIG_1      0x01
#define REG2_ACCEL_SMPLRT_DIV_1 0x10
#define REG2_ACCEL_SMPLRT_DIV_2 0x11
#define REG2_ACCEL_CONFIG       0x14

/* Bank 3 - 磁力计 */
#define REG3_I2C_MST_CTRL       0x01
#define REG3_I2C_SLV0_ADDR      0x03
#define REG3_I2C_SLV0_REG       0x04
#define REG3_I2C_SLV0_CTRL      0x05
#define REG3_I2C_SLV0_DO        0x06
#define REG3_I2C_SLV1_ADDR      0x07
#define REG3_I2C_SLV1_REG       0x08
#define REG3_I2C_SLV1_CTRL      0x09
#define REG3_EXT_SLV_SENS_DATA_00 0x3B

/* AK09916 磁力计地址 */
#define AK09916_I2C_ADDR        0x0C
#define AK09916_WIA2            0x01
#define AK09916_ST1             0x10
#define AK09916_HXL             0x11
#define AK09916_CNTL2           0x31

/* ============== 量程转换 ============== */

/* 加速度计灵敏度 (LSB/g) */
static const float ACCEL_SENS[] = {
    16384.0f,   /* ±2g */
    8192.0f,    /* ±4g */
    4096.0f,    /* ±8g */
    2048.0f     /* ±16g */
};

/* 陀螺仪灵敏度 (LSB/(deg/s)) */
static const float GYRO_SENS[] = {
    131.0f,     /* ±250 dps */
    65.5f,      /* ±500 dps */
    32.8f,      /* ±1000 dps */
    16.4f       /* ±2000 dps */
};

/* ============== 内部状态 ============== */

static struct {
    int i2c_handle;
    imu_config_t config;
    float gyro_offset[3];
    float accel_scale;
    float gyro_scale;
    bool initialized;
} g_imu;

/* ============== I2C 辅助 ============== */

static int bank_select(uint8_t bank)
{
    return i2cWriteByte(g_imu.i2c_handle, REG_BANK_SEL, (bank << 4) & 0xF0);
}

static int read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2cReadI2CBlockData(g_imu.i2c_handle, reg, (char *)buf, len);
}

static int write_reg(uint8_t reg, uint8_t val)
{
    return i2cWriteByte(g_imu.i2c_handle, reg, val);
}

static int16_t merge_bytes(uint8_t high, uint8_t low)
{
    return (int16_t)((high << 8) | low);
}

/* ============== 公开接口 ============== */

int imu_init(const imu_config_t *cfg)
{
    if (g_imu.initialized) return 0;

    memcpy(&g_imu.config, cfg, sizeof(imu_config_t));
    memset(g_imu.gyro_offset, 0, sizeof(g_imu.gyro_offset));

    /* 打开 I2C */
    g_imu.i2c_handle = i2cOpen(cfg->i2c_bus, cfg->i2c_addr, 0);
    if (g_imu.i2c_handle < 0) {
        fprintf(stderr, "[IMU] I2C open failed: bus=%d addr=0x%02x\n",
                cfg->i2c_bus, cfg->i2c_addr);
        return -1;
    }

    /* 复位 */
    bank_select(0);
    write_reg(REG_PWR_MGMT_1, 0x80);   /* DEVICE_RESET */
    usleep(50000);
    write_reg(REG_PWR_MGMT_1, 0x01);   /* CLKSEL=1 (PLL) */
    usleep(50000);

    /* 验证 WHO_AM_I */
    uint8_t who_am_i = 0;
    read_reg(REG_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != ICM20948_WHO_AM_I) {
        fprintf(stderr, "[IMU] WHO_AM_I mismatch: got 0x%02x, expected 0x%02x\n",
                who_am_i, ICM20948_WHO_AM_I);
        i2cClose(g_imu.i2c_handle);
        return -2;
    }
    printf("[IMU] ICM20948 detected (WHO_AM_I=0x%02x)\n", who_am_i);

    /* 配置加速度计 - Bank 2 */
    bank_select(2);

    /* 加速度计采样率 */
    uint16_t accel_div = (uint16_t)(1125.0f / cfg->sample_rate) - 1;
    write_reg(REG2_ACCEL_SMPLRT_DIV_1, (accel_div >> 8) & 0xFF);
    write_reg(REG2_ACCEL_SMPLRT_DIV_2, accel_div & 0xFF);

    /* 加速度计量程 */
    uint8_t accel_fs = 0;
    switch (cfg->accel_range) {
        case 2:  accel_fs = 0; break;
        case 4:  accel_fs = 1; break;
        case 8:  accel_fs = 2; break;
        case 16: accel_fs = 3; break;
        default: accel_fs = 1; break;
    }
    write_reg(REG2_ACCEL_CONFIG, (accel_fs << 1) | 0x01); /* ACCEL_FCHOICE=1 */
    g_imu.accel_scale = 1.0f / ACCEL_SENS[accel_fs];      /* LSB -> g */
    g_imu.accel_scale *= 9.80665f;                          /* g -> m/s² */

    /* 陀螺仪采样率 */
    uint8_t gyro_div = (uint8_t)(1100.0f / cfg->sample_rate) - 1;
    write_reg(REG2_GYRO_SMPLRT_DIV, gyro_div);

    /* 陀螺仪量程 */
    uint8_t gyro_fs = 0;
    switch (cfg->gyro_range) {
        case 250:  gyro_fs = 0; break;
        case 500:  gyro_fs = 1; break;
        case 1000: gyro_fs = 2; break;
        case 2000: gyro_fs = 3; break;
        default:   gyro_fs = 2; break;
    }
    write_reg(REG2_GYRO_CONFIG_1, (gyro_fs << 1) | 0x01);  /* GYRO_FCHOICE=1 */
    g_imu.gyro_scale = 1.0f / GYRO_SENS[gyro_fs];           /* LSB -> deg/s */

    /* 配置磁力计 - Bank 3 */
    bank_select(3);
    write_reg(REG3_I2C_MST_CTRL, 0x17);  /* I2C master clock ~400kHz */

    /* 设置 SLV0 读取磁力计 */
    write_reg(REG3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); /* read */
    write_reg(REG3_I2C_SLV0_REG, AK09916_ST1);
    write_reg(REG3_I2C_SLV0_CTRL, 0x89);  /* EN + 9 bytes */

    /* 设置 SLV1 写磁力计配置 */
    write_reg(REG3_I2C_SLV1_ADDR, AK09916_I2C_ADDR);        /* write */
    write_reg(REG3_I2C_SLV1_REG, AK09916_CNTL2);
    write_reg(REG3_I2C_SLV1_CTRL, 0x81);  /* EN + 1 byte */
    write_reg(REG3_I2C_SLV0_DO, 0x02);    /* Continuous mode 100Hz */

    /* 启用 I2C master */
    bank_select(0);
    write_reg(REG_USER_CTRL, 0x20);       /* I2C_MST_EN */
    usleep(10000);

    /* 启用加速度计和陀螺仪 */
    write_reg(REG_PWR_MGMT_2, 0x00);      /* 全部启用 */

    g_imu.initialized = true;
    printf("[IMU] Init OK: accel=±%dg, gyro=±%ddps, rate=%dHz\n",
           cfg->accel_range, cfg->gyro_range, cfg->sample_rate);
    return 0;
}

int imu_read(imu_data_t *data)
{
    if (!g_imu.initialized) return -1;

    uint8_t buf[20];

    /* 读加速度 (6 bytes) + 陀螺仪 (6 bytes) + 温度 (2 bytes) */
    bank_select(0);
    if (read_reg(REG_ACCEL_XOUT_H, buf, 14) < 0) return -2;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    data->timestamp = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;

    /* 加速度 */
    data->accel[0] = merge_bytes(buf[0], buf[1]) * g_imu.accel_scale;
    data->accel[1] = merge_bytes(buf[2], buf[3]) * g_imu.accel_scale;
    data->accel[2] = merge_bytes(buf[4], buf[5]) * g_imu.accel_scale;

    /* 陀螺仪 (减去偏移) */
    data->gyro[0] = merge_bytes(buf[6], buf[7])  * g_imu.gyro_scale - g_imu.gyro_offset[0];
    data->gyro[1] = merge_bytes(buf[8], buf[9])  * g_imu.gyro_scale - g_imu.gyro_offset[1];
    data->gyro[2] = merge_bytes(buf[10], buf[11]) * g_imu.gyro_scale - g_imu.gyro_offset[2];

    /* 温度 */
    int16_t temp_raw = merge_bytes(buf[12], buf[13]);
    data->temp = ((float)temp_raw / 333.87f) + 21.0f;

    /* 磁力计 (从 ext_sens_data) */
    if (read_reg(REG3_EXT_SLV_SENS_DATA_00, buf, 9) >= 0) {
        bank_select(0);
        data->mag[0] = (float)(int16_t)(buf[2] | (buf[3] << 8)); /* HXL */
        data->mag[1] = (float)(int16_t)(buf[4] | (buf[5] << 8)); /* HYL */
        data->mag[2] = (float)(int16_t)(buf[6] | (buf[7] << 8)); /* HZL */
    }

    return 0;
}

int imu_reset(void)
{
    if (!g_imu.initialized) return -1;
    bank_select(0);
    write_reg(REG_PWR_MGMT_1, 0x80);
    usleep(50000);
    write_reg(REG_PWR_MGMT_1, 0x01);
    usleep(50000);
    return 0;
}

int imu_self_test(void)
{
    uint8_t who_am_i = 0;
    bank_select(0);
    read_reg(REG_WHO_AM_I, &who_am_i, 1);
    return (who_am_i == ICM20948_WHO_AM_I) ? 0 : -1;
}

void imu_close(void)
{
    if (g_imu.initialized) {
        i2cClose(g_imu.i2c_handle);
        g_imu.initialized = false;
        printf("[IMU] Closed\n");
    }
}

int imu_calibrate(int samples)
{
    if (!g_imu.initialized) return -1;

    printf("[IMU] Calibrating (%d samples)... Keep IMU still!\n", samples);

    float sum[3] = {0};
    imu_data_t data;

    for (int i = 0; i < samples; i++) {
        if (imu_read(&data) < 0) return -2;
        sum[0] += data.gyro[0];
        sum[1] += data.gyro[1];
        sum[2] += data.gyro[2];
        usleep(5000); /* 5ms */
    }

    g_imu.gyro_offset[0] = sum[0] / samples;
    g_imu.gyro_offset[1] = sum[1] / samples;
    g_imu.gyro_offset[2] = sum[2] / samples;

    printf("[IMU] Calibration done: offset=[%.3f, %.3f, %.3f] deg/s\n",
           g_imu.gyro_offset[0], g_imu.gyro_offset[1], g_imu.gyro_offset[2]);
    return 0;
}

void imu_get_gyro_offset(float offset[3])
{
    memcpy(offset, g_imu.gyro_offset, sizeof(g_imu.gyro_offset));
}

void imu_set_gyro_offset(const float offset[3])
{
    memcpy(g_imu.gyro_offset, offset, sizeof(g_imu.gyro_offset));
}
