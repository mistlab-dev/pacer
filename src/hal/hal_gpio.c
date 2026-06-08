/**
 * @file hal_gpio.c
 * @brief GPIO/PWM 硬件抽象 — pigpio 实现
 */

#include "hal/hal_gpio.h"
#include <pigpio.h>

int hal_gpio_init(void)
{
    return gpioInitialise();
}

void hal_gpio_deinit(void)
{
    gpioTerminate();
}

void hal_gpio_set_mode(int pin, int mode)
{
    gpioSetMode(pin, mode ? PI_OUTPUT : PI_INPUT);
}

void hal_gpio_write(int pin, int value)
{
    gpioWrite(pin, value);
}

int hal_gpio_read(int pin)
{
    return gpioRead(pin);
}

int hal_pwm_set_range(int pin, int range)
{
    return gpioSetPWMrange(pin, range);
}

int hal_pwm_set_freq(int pin, int freq)
{
    return gpioSetPWMfrequency(pin, freq);
}

void hal_pwm_write(int pin, int duty)
{
    gpioPWM(pin, duty);
}

void hal_delay_us(unsigned int us)
{
    gpioDelay(us);
}
