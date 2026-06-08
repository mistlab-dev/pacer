/**
 * @file hal_pca9685.h
 * @brief PCA9685 I2C PWM 扩展板驱动
 *
 * 16 路 PWM，I2C 控制，适合驱动多个电机/舵机。
 * 树莓派通过 I2C 连接 PCA9685，
 * 上层通过 hal_pwm_write() 统一调用。
 */

#ifndef PACER_HAL_PCA9685_H
#define PACER_HAL_PCA9685_H

#include <stdint.h>
#include <stdbool.h>

#define PCA9685_ADDR_DEFAULT    0x40
#define PCA9685_CHANNELS        16

/**
 * @brief 初始化 PCA9685
 * @param i2c_bus  I2C 总线号
 * @param addr     I2C 地址 (默认 0x40)
 * @param freq_hz  PWM 频率 (电机用 20kHz, 舵机用 50Hz)
 * @return 0=成功
 */
int  hal_pca9685_init(int i2c_bus, uint8_t addr, int freq_hz);

/**
 * @brief 设置单个通道 PWM 占空比
 * @param channel  0~15
 * @param duty     0~pwm_range
 * @param pwm_range 分辨率 (如 4095)
 */
void hal_pca9685_set_pwm(int channel, uint16_t duty, uint16_t pwm_range);

/**
 * @brief 关闭 PCA9685 (所有通道归零)
 */
void hal_pca9685_deinit(void);

/** 获取当前 I2C 文件描述符 (调试用) */
int  hal_pca9685_get_fd(void);

#endif /* PACER_HAL_PCA9685_H */
