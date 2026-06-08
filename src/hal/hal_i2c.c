/**
 * @file hal_i2c.c
 * @brief I2C 硬件抽象 — pigpio 实现
 */

#include "hal/hal_i2c.h"
#include <pigpio.h>
#include <stdio.h>

int i2c_open(i2c_dev_t *dev, int bus, uint8_t addr)
{
    dev->bus  = bus;
    dev->addr = addr;
    dev->fd   = i2cOpen(bus, addr, 0);

    if (dev->fd < 0) {
        fprintf(stderr, "[HAL] I2C open failed: bus=%d addr=0x%02x\n", bus, addr);
        dev->open = false;
        return -1;
    }

    dev->open = true;
    return 0;
}

void i2c_close(i2c_dev_t *dev)
{
    if (dev->open) {
        i2cClose(dev->fd);
        dev->open = false;
    }
}

int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2cWriteByteData(dev->fd, reg, val);
}

int i2c_read_regs(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2cReadI2CBlockData(dev->fd, reg, (char *)buf, len);
}

int i2c_read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    int r = i2cReadByteData(dev->fd, reg);
    if (r < 0) return r;
    *val = (uint8_t)r;
    return 0;
}
