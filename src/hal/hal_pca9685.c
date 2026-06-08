/**
 * @file hal_pca9685.c
 * @brief PCA9685 I2C PWM 扩展板驱动实现
 *
 * PCA9685 寄存器:
 *   MODE1      0x00
 *   MODE2      0x01
 *   PRE_SCALE  0xFE
 *   LEDx_ON_L  0x06 + 4*ch
 *   LEDx_OFF_L 0x08 + 4*ch
 *
 * PWM 频率 = osc_clock / (4096 * prescale)
 * osc_clock = 25MHz (内部振荡器)
 */

#include "hal/hal_pca9685.h"
#include "hal/hal_i2c.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>

/* PCA9685 寄存器 */
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_PRESCALE    0xFE
#define REG_LED0_ON_L   0x06

/* MODE1 位 */
#define MODE1_RESTART   (1 << 7)
#define MODE1_SLEEP     (1 << 4)
#define MODE1_AI        (1 << 5)  /* auto-increment */

/* 振荡器频率 */
#define OSC_FREQ        25000000.0

static i2c_dev_t g_dev;

/* ---------- 公开接口 ---------- */

int hal_pca9685_init(int i2c_bus, uint8_t addr, int freq_hz)
{
    memset(&g_dev, 0, sizeof(g_dev));

    /* 打开 I2C 设备 */
    if (i2c_open(&g_dev, i2c_bus, addr) != 0) {
        fprintf(stderr, "[PCA9685] failed to open I2C bus %d addr 0x%02X\n",
                i2c_bus, addr);
        return -1;
    }

    /* 进入 sleep 模式才能设置频率 */
    i2c_write_reg(&g_dev, REG_MODE1, MODE1_SLEEP);

    /* 计算 prescale: freq = OSC / (4096 * prescale) */
    float prescale_f = OSC_FREQ / (4096.0f * (float)freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
    if (prescale < 3) prescale = 3;  /* 最小值 3 */

    i2c_write_reg(&g_dev, REG_PRESCALE, prescale);

    /* 唤醒, 启用 auto-increment */
    i2c_write_reg(&g_dev, REG_MODE1, MODE1_AI);

    /* 等待振荡器稳定 */
    usleep(500);

    /* 重启 */
    uint8_t mode1;
    i2c_read_reg(&g_dev, REG_MODE1, &mode1);
    i2c_write_reg(&g_dev, REG_MODE1, mode1 | MODE1_RESTART);

    /* MODE2: 推挽输出 */
    i2c_write_reg(&g_dev, REG_MODE2, 0x04);

    /* 所有通道归零: ON=0, OFF=0 */
    for (int i = 0; i < PCA9685_CHANNELS; i++) {
        uint8_t buf[5];
        uint8_t base = REG_LED0_ON_L + 4 * i;
        buf[0] = base;
        buf[1] = 0; buf[2] = 0;  /* ON = 0 */
        buf[3] = 0; buf[4] = 0;  /* OFF = 0 */
        /* 用 i2c_read_regs 写需要直接操作; 这里用单字节写模拟 */
        i2c_write_reg(&g_dev, base,     0);  /* ON_L */
        i2c_write_reg(&g_dev, base + 1, 0);  /* ON_H */
        i2c_write_reg(&g_dev, base + 2, 0);  /* OFF_L */
        i2c_write_reg(&g_dev, base + 3, 0);  /* OFF_H */
    }

    printf("[PCA9685] init bus=%d addr=0x%02X freq=%dHz prescale=%d\n",
           i2c_bus, addr, freq_hz, prescale);
    return 0;
}

void hal_pca9685_set_pwm(int channel, uint16_t duty, uint16_t pwm_range)
{
    if (!g_dev.open || channel < 0 || channel >= PCA9685_CHANNELS) return;

    /* 映射到 0~4095 */
    uint16_t off = 0;
    if (duty > 0 && pwm_range > 0) {
        off = (uint16_t)((uint32_t)duty * 4095 / pwm_range);
        if (off > 4095) off = 4095;
    }

    uint8_t base = REG_LED0_ON_L + 4 * channel;
    /* ON = 0 (从周期起点开始) */
    i2c_write_reg(&g_dev, base,     0);
    i2c_write_reg(&g_dev, base + 1, 0);
    /* OFF = off */
    i2c_write_reg(&g_dev, base + 2, (uint8_t)(off & 0xFF));
    i2c_write_reg(&g_dev, base + 3, (uint8_t)(off >> 8));
}

void hal_pca9685_deinit(void)
{
    if (!g_dev.open) return;

    /* 所有通道关闭 */
    for (int i = 0; i < PCA9685_CHANNELS; i++) {
        hal_pca9685_set_pwm(i, 0, 4095);
    }

    /* 进入 sleep */
    i2c_write_reg(&g_dev, REG_MODE1, MODE1_SLEEP);
    i2c_close(&g_dev);
    printf("[PCA9685] deinit\n");
}

int hal_pca9685_get_fd(void)
{
    return g_dev.fd;
}
