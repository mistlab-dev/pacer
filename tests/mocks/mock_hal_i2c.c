/**
 * @file mock_hal_i2c.c
 * @brief I2C HAL mock — 模拟 PCA9685 寄存器
 */

#include "hal/hal_i2c.h"
#include <string.h>
#include <stdio.h>

/* 模拟 PCA9685 寄存器空间 */
#define MOCK_REG_SIZE 256
static uint8_t g_regs[MOCK_REG_SIZE];
static int g_i2c_open_called = 0;

int i2c_open(i2c_dev_t *dev, int bus, uint8_t addr)
{
    dev->bus  = bus;
    dev->addr = addr;
    dev->fd   = 1;  /* fake fd */
    dev->open = true;
    g_i2c_open_called = 1;
    memset(g_regs, 0, sizeof(g_regs));
    return 0;
}

void i2c_close(i2c_dev_t *dev)
{
    dev->open = false;
}

int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    (void)dev;
    if (reg < MOCK_REG_SIZE) {
        g_regs[reg] = val;
        return 0;
    }
    return -1;
}

int i2c_read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    (void)dev;
    if (reg < MOCK_REG_SIZE) {
        *val = g_regs[reg];
        return 0;
    }
    return -1;
}

int i2c_read_regs(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    (void)dev;
    if (reg + len <= MOCK_REG_SIZE) {
        memcpy(buf, &g_regs[reg], len);
        return 0;
    }
    return -1;
}

/* ---- 测试辅助 ---- */

void mock_i2c_reset(void)
{
    memset(g_regs, 0, sizeof(g_regs));
    g_i2c_open_called = 0;
}

uint8_t mock_i2c_get_reg(uint8_t reg) { return g_regs[reg]; }
int mock_i2c_was_opened(void) { return g_i2c_open_called; }
