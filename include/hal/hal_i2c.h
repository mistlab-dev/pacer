/**
 * @file hal_i2c.h
 * @brief I2C 硬件抽象层
 *
 * 封装底层 I2C 操作，方便移植到不同平台。
 * 树莓派上基于 pigpio 实现。
 */

#ifndef PACER_HAL_I2C_H
#define PACER_HAL_I2C_H

#include <stdint.h>
#include <stdbool.h>

/* I2C 设备句柄 */
typedef struct {
    int     fd;         /* pigpio i2c handle */
    int     bus;        /* I2C 总线号 */
    uint8_t addr;       /* 设备地址 (7-bit) */
    bool    open;       /* 是否已打开 */
} i2c_dev_t;

/**
 * @brief 打开 I2C 设备
 * @param dev  设备句柄
 * @param bus  总线号 (如 1)
 * @param addr 7-bit 设备地址
 * @return 0=成功, -1=失败
 */
int  i2c_open(i2c_dev_t *dev, int bus, uint8_t addr);

/**
 * @brief 关闭 I2C 设备
 */
void i2c_close(i2c_dev_t *dev);

/**
 * @brief 写 1 字节到寄存器
 */
int  i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief 从寄存器读取多个字节
 */
int  i2c_read_regs(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief 读 1 字节寄存器
 */
int  i2c_read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val);

#endif /* PACER_HAL_I2C_H */
