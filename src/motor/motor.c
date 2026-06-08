/**
 * @file motor.c
 * @brief 统一电机驱动 — 支持 GPIO PWM 和 PCA9685
 *
 * 驱动方式:
 *   pin_a = PWM, pin_b = 0  →  正转
 *   pin_a = 0,   pin_b = PWM →  反转
 *   pin_a = 0,   pin_b = 0   →  滑行
 *   两者同时高                  →  刹车
 */

#include "motor/motor.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pca9685.h"
#include "app/config.h"

#include <stdio.h>
#include <math.h>

/* ---------- PCA9685 辅助 ---------- */

static void pca_write_pwm(int channel, int duty)
{
    hal_pca9685_set_pwm(channel, (uint16_t)duty, (uint16_t)CFG_MOTOR_PWM_RANGE);
}

/* ---------- 公开接口 ---------- */

int motor_init(motor_t *m, motor_pwm_src_t pwm_src,
               int pin_a, int pin_b, int pwm_range, bool inverted)
{
    m->pwm_src   = pwm_src;
    m->pin_a     = pin_a;
    m->pin_b     = pin_b;
    m->pwm_range = pwm_range;
    m->inverted  = inverted;
    m->ready     = false;

    if (pwm_src == MOTOR_PWM_GPIO) {
        hal_gpio_set_mode(pin_a, 1);
        hal_gpio_set_mode(pin_b, 1);
        hal_pwm_set_range(pin_a, pwm_range);
        hal_pwm_set_freq(pin_a, CFG_MOTOR_PWM_HZ);
        hal_pwm_set_range(pin_b, pwm_range);
        hal_pwm_set_freq(pin_b, CFG_MOTOR_PWM_HZ);
        hal_pwm_write(pin_a, 0);
        hal_pwm_write(pin_b, 0);
    }
    /* PCA9685 不需要逐通道初始化，由 hal_pca9685_init 统一完成 */
    else if (pwm_src == MOTOR_PWM_PCA9685) {
        pca_write_pwm(pin_a, 0);
        pca_write_pwm(pin_b, 0);
    }

    m->ready = true;
    printf("[MOTOR] src=%s pins=%d,%d range=%d inv=%s\n",
           pwm_src == MOTOR_PWM_GPIO ? "GPIO" : "PCA9685",
           pin_a, pin_b, pwm_range, inverted ? "yes" : "no");
    return 0;
}

void motor_set(motor_t *m, float power)
{
    if (!m->ready) return;

    if (m->inverted) power = -power;

    if (power >  (float)m->pwm_range) power =  (float)m->pwm_range;
    if (power < -(float)m->pwm_range) power = -(float)m->pwm_range;

    int duty = (int)fabsf(power);

    if (m->pwm_src == MOTOR_PWM_GPIO) {
        if (power > 0.0f) {
            hal_pwm_write(m->pin_a, duty);
            hal_pwm_write(m->pin_b, 0);
        } else if (power < 0.0f) {
            hal_pwm_write(m->pin_a, 0);
            hal_pwm_write(m->pin_b, duty);
        } else {
            hal_pwm_write(m->pin_a, 0);
            hal_pwm_write(m->pin_b, 0);
        }
    } else {
        if (power > 0.0f) {
            pca_write_pwm(m->pin_a, duty);
            pca_write_pwm(m->pin_b, 0);
        } else if (power < 0.0f) {
            pca_write_pwm(m->pin_a, 0);
            pca_write_pwm(m->pin_b, duty);
        } else {
            pca_write_pwm(m->pin_a, 0);
            pca_write_pwm(m->pin_b, 0);
        }
    }
}

void motor_brake(motor_t *m)
{
    if (!m->ready) return;
    if (m->pwm_src == MOTOR_PWM_GPIO) {
        hal_pwm_write(m->pin_a, m->pwm_range);
        hal_pwm_write(m->pin_b, m->pwm_range);
    } else {
        pca_write_pwm(m->pin_a, m->pwm_range);
        pca_write_pwm(m->pin_b, m->pwm_range);
    }
}

void motor_deinit(motor_t *m)
{
    if (!m->ready) return;
    if (m->pwm_src == MOTOR_PWM_GPIO) {
        hal_pwm_write(m->pin_a, 0);
        hal_pwm_write(m->pin_b, 0);
    } else {
        pca_write_pwm(m->pin_a, 0);
        pca_write_pwm(m->pin_b, 0);
    }
    m->ready = false;
    printf("[MOTOR] pins=%d,%d closed\n", m->pin_a, m->pin_b);
}
