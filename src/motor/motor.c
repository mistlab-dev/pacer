/**
 * @file motor.c
 * @brief 双向 PWM 电机驱动
 *
 * 驱动方式:
 *   pin_a = PWM, pin_b = 0  →  正转
 *   pin_a = 0,   pin_b = PWM →  反转
 *   pin_a = 0,   pin_b = 0   →  滑行 (自由停)
 *   两者同时高                  →  刹车 (急停)
 *
 * inverted 标志用于物理安装方向不一致时,
 * 在软件层反转, 不改接线。
 */

#include "motor/motor.h"
#include "hal/hal_gpio.h"
#include "app/config.h"

#include <stdio.h>
#include <math.h>

int motor_init(motor_t *m, int pin_a, int pin_b, int pwm_range, bool inverted)
{
    m->pin_a     = pin_a;
    m->pin_b     = pin_b;
    m->pwm_range = pwm_range;
    m->inverted  = inverted;
    m->ready     = false;

    /* 配置引脚 */
    hal_gpio_set_mode(pin_a, 1);  /* output */
    hal_gpio_set_mode(pin_b, 1);

    hal_pwm_set_range(pin_a, pwm_range);
    hal_pwm_set_freq(pin_a, CFG_MOTOR_PWM_HZ);
    hal_pwm_set_range(pin_b, pwm_range);
    hal_pwm_set_freq(pin_b, CFG_MOTOR_PWM_HZ);

    /* 初始停止 */
    hal_pwm_write(pin_a, 0);
    hal_pwm_write(pin_b, 0);

    m->ready = true;
    printf("[MOTOR] pins=%d,%d range=%d inverted=%s\n",
           pin_a, pin_b, pwm_range, inverted ? "yes" : "no");
    return 0;
}

void motor_set(motor_t *m, float power)
{
    if (!m->ready) return;

    /* 方向反转 */
    if (m->inverted) power = -power;

    /* 限幅 */
    if (power >  (float)m->pwm_range) power =  (float)m->pwm_range;
    if (power < -(float)m->pwm_range) power = -(float)m->pwm_range;

    int duty = (int)fabsf(power);

    if (power > 0.0f) {
        hal_pwm_write(m->pin_a, duty);   /* 正转 */
        hal_pwm_write(m->pin_b, 0);
    } else if (power < 0.0f) {
        hal_pwm_write(m->pin_a, 0);
        hal_pwm_write(m->pin_b, duty);   /* 反转 */
    } else {
        hal_pwm_write(m->pin_a, 0);      /* 滑行 */
        hal_pwm_write(m->pin_b, 0);
    }
}

void motor_brake(motor_t *m)
{
    if (!m->ready) return;
    hal_pwm_write(m->pin_a, m->pwm_range);
    hal_pwm_write(m->pin_b, m->pwm_range);
}

void motor_deinit(motor_t *m)
{
    if (!m->ready) return;
    hal_pwm_write(m->pin_a, 0);
    hal_pwm_write(m->pin_b, 0);
    m->ready = false;
    printf("[MOTOR] pins=%d,%d closed\n", m->pin_a, m->pin_b);
}
