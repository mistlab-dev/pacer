/**
 * @file motor.c
 * @brief 统一电机驱动 — 支持 DC (GPIO/PCA9685) 和 ESC (电调)
 *
 * DC 模式:
 *   pin_a = PWM, pin_b = 0  →  正转
 *   pin_a = 0,   pin_b = PWM →  反转
 *   pin_a = 0,   pin_b = 0   →  滑行
 *
 * ESC 模式:
 *   舵机信号 50Hz, 脉宽 1000~2000μs
 *   power -1.0 ~ +1.0 → 脉宽 1000~2000μs
 *   0.0 = 1500μs (中位/停止)
 *   +1.0 = 2000μs (正转最大)
 *   -1.0 = 1000μs (反转最大, 需电调支持双向)
 */

#include "motor/motor.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pca9685.h"
#include "app/config.h"

#include <stdio.h>
#include <math.h>

/* ESC 脉宽范围 (μs) */
#define ESC_PULSE_MIN     1000    /* 反转最大 / 停止 */
#define ESC_PULSE_MID     1500    /* 中位 (停止) */
#define ESC_PULSE_MAX     2000    /* 正转最大 */

/*
 * ESC: power (0~1) → PCA9685 duty
 *
 * PCA9685 分辨率固定 4096。
 * 不管 PCA9685 的 PWM 频率设多少，tick 的计算方式都一样:
 *   duty = pulse_us * 4096 / period_us
 *
 * 对于标准 ESC (1000~2000μs):
 *   0.0 → 1000μs → 1000 * 4096 / period
 *   1.0 → 2000μs → 2000 * 4096 / period
 *
 * 注意: period 随 PWM 频率变化:
 *   50Hz  → period = 20000μs → 1000μs ≈ 205 ticks, 2000μs ≈ 410 ticks
 *   400Hz → period = 2500μs  → 1000μs ≈ 1638 ticks, 2000μs ≈ 3277 ticks
 */
static void pca_write_pwm(int channel, int duty)
{
    hal_pca9685_set_pwm(channel, (uint16_t)duty, (uint16_t)CFG_MOTOR_PWM_RANGE);
}

static uint16_t esc_power_to_duty(float power)
{
    if (power >  1.0f) power =  1.0f;
    if (power <  0.0f) power =  0.0f;  /* 四旋翼不允许负油门 */

    /* power: 0~1 → pulse: 1000~2000μs */
    float pulse_us = ESC_PULSE_MIN + power * (ESC_PULSE_MAX - ESC_PULSE_MIN);

    /* μs → PCA9685 ticks (4096 级), period 由 PCA9685 频率决定 */
    float period_us = 1000000.0f / (float)CFG_PCA9685_PWM_HZ;
    uint16_t ticks = (uint16_t)(pulse_us * 4096.0f / period_us);

    return ticks;
}

/* ---------- 公开接口 ---------- */

int motor_init_dc(motor_t *m, motor_type_t type,
                  int pin_a, int pin_b, int pwm_range, bool inverted)
{
    m->type     = type;
    m->pin_a    = pin_a;
    m->pin_b    = pin_b;
    m->pwm_range = pwm_range;
    m->inverted = inverted;
    m->armed    = false;
    m->ready    = false;

    if (type == MOTOR_TYPE_DC_GPIO) {
        hal_gpio_set_mode(pin_a, 1);
        hal_gpio_set_mode(pin_b, 1);
        hal_pwm_set_range(pin_a, pwm_range);
        hal_pwm_set_freq(pin_a, CFG_MOTOR_PWM_HZ);
        hal_pwm_set_range(pin_b, pwm_range);
        hal_pwm_set_freq(pin_b, CFG_MOTOR_PWM_HZ);
        hal_pwm_write(pin_a, 0);
        hal_pwm_write(pin_b, 0);
    } else {
        pca_write_pwm(pin_a, 0);
        pca_write_pwm(pin_b, 0);
    }

    m->ready = true;
    printf("[MOTOR] DC type=%s pins=%d,%d range=%d inv=%s\n",
           type == MOTOR_TYPE_DC_GPIO ? "GPIO" : "PCA9685",
           pin_a, pin_b, pwm_range, inverted ? "yes" : "no");
    return 0;
}

int motor_init_esc(motor_t *m, int signal_ch, bool inverted)
{
    m->type      = MOTOR_TYPE_ESC;
    m->pin_a     = signal_ch;
    m->pin_b     = -1;
    m->pwm_range = CFG_MOTOR_PWM_RANGE;
    m->inverted  = inverted;
    m->armed     = false;
    m->ready     = false;

    /* 初始输出中位 (1500μs) — 安全 */
    hal_pca9685_set_pwm(signal_ch, esc_power_to_duty(0.0f), 4095);

    m->ready = true;
    printf("[MOTOR] ESC ch=%d inv=%s\n", signal_ch, inverted ? "yes" : "no");
    return 0;
}

void motor_set(motor_t *m, float power)
{
    if (!m->ready) return;

    /* ---- ESC 模式 (四旋翼: 0~1, 不允许负油门) ---- */
    if (m->type == MOTOR_TYPE_ESC) {
        if (!m->armed) return;
        /* 四旋翼不需要 inversion，混控器已经处理了方向 */
        uint16_t ticks = esc_power_to_duty(power);
        hal_pca9685_set_pwm(m->pin_a, ticks, 4095);
        return;
    }

    /* ---- DC 模式 ---- */
    if (m->inverted) power = -power;

    if (power >  (float)m->pwm_range) power =  (float)m->pwm_range;
    if (power < -(float)m->pwm_range) power = -(float)m->pwm_range;

    int duty = (int)fabsf(power);

    if (m->type == MOTOR_TYPE_DC_GPIO) {
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
    } else {  /* DC_PCA9685 */
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

    if (m->type == MOTOR_TYPE_ESC) {
        /* ESC 没有传统刹车, 输出中位停止 */
        hal_pca9685_set_pwm(m->pin_a, esc_power_to_duty(0.0f), 4095);
    } else if (m->type == MOTOR_TYPE_DC_GPIO) {
        hal_pwm_write(m->pin_a, m->pwm_range);
        hal_pwm_write(m->pin_b, m->pwm_range);
    } else {
        pca_write_pwm(m->pin_a, m->pwm_range);
        pca_write_pwm(m->pin_b, m->pwm_range);
    }
}

void motor_esc_arm(motor_t *m)
{
    if (m->type != MOTOR_TYPE_ESC || !m->ready) return;

    /* 发送最小油门 1000μs — 四旋翼电调看到最低油门完成初始化 */
    uint16_t min_ticks = esc_power_to_duty(0.0f);
    hal_pca9685_set_pwm(m->pin_a, min_ticks, 4095);

    m->armed = true;
    printf("[MOTOR] ESC ch=%d ARMED (1000μs)\n", m->pin_a);
}

void motor_deinit(motor_t *m)
{
    if (!m->ready) return;

    if (m->type == MOTOR_TYPE_ESC) {
        hal_pca9685_set_pwm(m->pin_a, esc_power_to_duty(0.0f), 4095);
    } else if (m->type == MOTOR_TYPE_DC_GPIO) {
        hal_pwm_write(m->pin_a, 0);
        hal_pwm_write(m->pin_b, 0);
    } else {
        pca_write_pwm(m->pin_a, 0);
        pca_write_pwm(m->pin_b, 0);
    }

    m->ready = false;
    printf("[MOTOR] pin=%d closed\n", m->pin_a);
}
