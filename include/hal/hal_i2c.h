/**
 * @file hal_i2c.h
 * @brief I2C 硬件抽象层 — STM32H743 HAL 实现
 *
 * 封装 STM32 HAL I2C 操作，保持与上层接口一致。
 * 底层基于 HAL_I2C_Master_Transmit / Receive。
 */

#ifndef PACER_HAL_I2C_H
#define PACER_HAL_I2C_H

#include <stdint.h>
#include <stdbool.h>

/* I2C 设备句柄 */
typedef struct {
    void    *hi2c;      /* STM32 I2C_HandleTypeDef* */
    uint8_t  addr;      /* 设备地址 (7-bit, 左移前) */
    bool     open;      /* 是否已打开 */
} i2c_dev_t;

/**
 * @brief 初始化 I2C1 外设 (400kHz Fast-mode)
 *        在 i2c_open 之前调用。
 */
void hal_i2c1_init(void);

/**
 * @brief 打开 I2C 设备
 * @param dev  设备句柄
 * @param bus  总线号 ( unused — STM32 通过 HAL handle 决定 )
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
