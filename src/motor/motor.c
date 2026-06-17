/**
 * @file motor.c
 * @brief 统一电机驱动 — ESC (硬件 Timer PWM)
 *
 * STM32H743 版本: 去掉 PCA9685 和 GPIO DC 电机支持，
 * 专注四旋翼 ESC 驱动。
 *
 * ESC 模式:
 *   50Hz 舵机信号, 脉宽 1000~2000μs
 *   power 0.0 ~ 1.0 → 脉宽 1000~2000μs
 *   0.0 = 1000μs (停止/最低)
 *   1.0 = 2000μs (最大油门)
 *
 * 解锁 (arm):
 *   输出 1000μs 保持 >1 秒, 大多数 ESC 会发出两声短音表示解锁。
 */

#include "motor/motor.h"
#include "hal/hal_tim.h"
#include "app/config.h"

#include <stdio.h>

/* ESC arm 状态 */
static bool esc_armed = false;

/* ================ 初始化 ================ */

int motor_init(void)
{
    /* Timer PWM 已由 hal_tim_pwm_init() 初始化
     * 这里只确保所有通道输出最低油门 */
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        hal_tim_pwm_set_pulse(i, CFG_ESC_PULSE_MIN);
    }
    esc_armed = false;
    return 0;
}

void motor_deinit(void)
{
    motor_disarm();
    hal_tim_pwm_stop_all();
}

/* ================ ESC 控制 ================ */

void motor_set(int channel, float power)
{
    if (!esc_armed) return;

    /* 钳位 */
    if (power < 0.0f) power = 0.0f;
    if (power > 1.0f) power = 1.0f;

    /* power 0~1 → 脉宽 1000~2000μs */
    uint16_t pulse = (uint16_t)(CFG_ESC_PULSE_MIN +
                                power * (CFG_ESC_PULSE_MAX - CFG_ESC_PULSE_MIN));
    hal_tim_pwm_set_pulse(channel, pulse);
}

void motor_set_all(float power)
{
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        motor_set(i, power);
    }
}

void motor_stop_all(void)
{
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        hal_tim_pwm_set_pulse(i, CFG_ESC_PULSE_MIN);
    }
}

/* ================ 解锁 ================ */

int motor_arm(void)
{
    if (esc_armed) return 0;

    /* 输出最低油门信号 — ESC 解锁 */
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        hal_tim_pwm_set_pulse(i, CFG_ESC_PULSE_ARM);
    }

    esc_armed = true;
    return 0;
}

void motor_disarm(void)
{
    motor_stop_all();
    esc_armed = false;
}

bool motor_is_armed(void)
{
    return esc_armed;
}
