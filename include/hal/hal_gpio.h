/**
 * @file hal_gpio.h
 * @brief GPIO/PWM 硬件抽象层
 *
 * 封装底层 GPIO 和 PWM 操作。
 */

#ifndef PACER_HAL_GPIO_H
#define PACER_HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>

/* 全局初始化 / 关闭 */
int  hal_gpio_init(void);
void hal_gpio_deinit(void);

/* GPIO */
void hal_gpio_set_mode(int pin, int mode);  /* 0=input, 1=output */
void hal_gpio_write(int pin, int value);
int  hal_gpio_read(int pin);

/* PWM */
int  hal_pwm_set_range(int pin, int range);
int  hal_pwm_set_freq(int pin, int freq);
void hal_pwm_write(int pin, int duty);

/* 微秒延时 */
void hal_delay_us(unsigned int us);

#endif /* PACER_HAL_GPIO_H */
