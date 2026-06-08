/**
 * @file motor.c
 * @brief 电机驱动实现 (pigpio 硬件 PWM)
 *
 * 双引脚驱动方式:
 *   pin_a = PWM, pin_b = 0  -> 正转
 *   pin_a = 0,   pin_b = PWM -> 反转
 *   pin_a = 0,   pin_b = 0   -> 滑行
 *   pin_a = 1,   pin_b = 1   -> 刹车
 */

#include "drivers/motor.h"
#include "config.h"
#include <pigpio.h>
#include <stdio.h>
#include <math.h>

int motor_init(motor_t *m, int pin_a, int pin_b)
{
    m->pin_a = pin_a;
    m->pin_b = pin_b;
    m->pwm_range = MOTOR_PWM_RANGE;
    m->pwm_freq = MOTOR_PWM_FREQ;
    m->initialized = false;

    /* 设置引脚为输出 */
    gpioSetMode(pin_a, PI_OUTPUT);
    gpioSetMode(pin_b, PI_OUTPUT);

    /* 配置硬件 PWM */
    gpioSetPWMfrequency(pin_a, m->pwm_freq);
    gpioSetPWMrange(pin_a, m->pwm_range);
    gpioSetPWMfrequency(pin_b, m->pwm_freq);
    gpioSetPWMrange(pin_b, m->pwm_range);

    /* 初始状态: 停止 */
    gpioPWM(pin_a, 0);
    gpioPWM(pin_b, 0);

    m->initialized = true;
    printf("[MOT] Motor initialized: pins=%d,%d, freq=%dHz, range=%d\n",
           pin_a, pin_b, m->pwm_freq, m->pwm_range);
    return 0;
}

void motor_set(motor_t *m, float power)
{
    if (!m->initialized) return;

    /* 限幅 */
    if (power > m->pwm_range) power = m->pwm_range;
    if (power < -m->pwm_range) power = -m->pwm_range;

    int duty = (int)fabsf(power);

    if (power > 0) {
        /* 正转 */
        gpioPWM(m->pin_a, duty);
        gpioPWM(m->pin_b, 0);
    } else if (power < 0) {
        /* 反转 */
        gpioPWM(m->pin_a, 0);
        gpioPWM(m->pin_b, duty);
    } else {
        /* 停止 */
        gpioPWM(m->pin_a, 0);
        gpioPWM(m->pin_b, 0);
    }
}

void motor_brake(motor_t *m)
{
    if (!m->initialized) return;
    gpioPWM(m->pin_a, m->pwm_range);
    gpioPWM(m->pin_b, m->pwm_range);
}

void motor_close(motor_t *m)
{
    if (!m->initialized) return;
    gpioPWM(m->pin_a, 0);
    gpioPWM(m->pin_b, 0);
    m->initialized = false;
    printf("[MOT] Motor closed: pins=%d,%d\n", m->pin_a, m->pin_b);
}
