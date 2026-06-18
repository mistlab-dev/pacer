/**
 * @file hal_gpio.h
 * @brief GPIO 硬件抽象层 — STM32H743
 *
 * 封装 STM32 GPIO 操作。
 * PWM 通过硬件 Timer 实现，见 hal_tim.h。
 */

#ifndef PACER_HAL_GPIO_H
#define PACER_HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* 全局初始化 / 关闭 (STM32 上由 CubeMX 生成的 MSP 初始化，这里为空操作) */
int  hal_gpio_init(void);
void hal_gpio_deinit(void);

/* GPIO — pin 编码: port * 16 + pin_number (PA0=0, PA1=1, ... PB0=16, etc.) */
void hal_gpio_set_mode(int pin, int mode);  /* 0=input, 1=output */
void hal_gpio_write(int pin, int value);
int  hal_gpio_read(int pin);

/* 微秒延时 (基于 DWT 计数器) */
void hal_delay_us(unsigned int us);

/* 毫秒延时 */
void hal_delay_ms(unsigned int ms);

/* 获取单调微秒时间戳 (基于 DWT) */
unsigned int hal_micros(void);
uint64_t     hal_micros_64(void);  /* 64-bit, 防 32-bit 溢出 */

#endif /* PACER_HAL_GPIO_H */
