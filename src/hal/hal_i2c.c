/**
 * @file hal_i2c.c
 * @brief I2C 硬件抽象 — STM32H743 HAL 实现
 *
 * 上层通过 i2c_open/write_reg/read_regs 统一调用，
 * 底层由 STM32 HAL 库驱动。
 * I2C1: PB6(SCL) / PB7(SDA), 400kHz Fast-mode
 */

#include "hal/hal_i2c.h"
#include "app/config.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* I2C1 全局句柄 */
I2C_HandleTypeDef hi2c1;

/* ---- I2C1 外设初始化 (由 app_init 调用) ---- */
void hal_i2c1_init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00C04E6E;  /* 400kHz Fast-mode, PCLK1=64MHz */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        /* 初始化失败 — 调用方检查设备就绪时会发现 */
    }
}

/* I2C 设备对应的 bus → STM32 只有一个 I2C1 接 IMU */
static I2C_HandleTypeDef *bus_to_handle(int bus)
{
    (void)bus;  /* STM32H743 pacer 只用 I2C1 */
    return &hi2c1;
}

int i2c_open(i2c_dev_t *dev, int bus, uint8_t addr)
{
    dev->addr  = addr << 1;   /* STM32 HAL 需要 8-bit 地址 (左移) */
    dev->hi2c  = bus_to_handle(bus);
    dev->open  = true;

    /* 测试通信 — 读 WHO_AM_I */
    HAL_StatusTypeDef st;
    st = HAL_I2C_IsDeviceReady(dev->hi2c, dev->addr, 3, 100);
    if (st != HAL_OK) {
        dev->open = false;
        return -1;
    }

    return 0;
}

void i2c_close(i2c_dev_t *dev)
{
    dev->open = false;
    dev->hi2c = NULL;
}

int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev->open) return -1;

    uint8_t buf[2] = { reg, val };
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
        dev->hi2c, dev->addr, buf, 2, 100);

    return (st == HAL_OK) ? 0 : -1;
}

int i2c_read_regs(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!dev->open) return -1;

    /* 先写寄存器地址 */
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
        dev->hi2c, dev->addr, &reg, 1, 100);
    if (st != HAL_OK) return -1;

    /* 再读数据 */
    st = HAL_I2C_Master_Receive(
        dev->hi2c, dev->addr, buf, len, 200);

    return (st == HAL_OK) ? len : -1;
}

int i2c_read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_read_regs(dev, reg, val, 1);
}
