/**
 * @file mock_hal_gpio.c
 * @brief GPIO/PWM HAL mock — 记录调用，不操作硬件
 */

#include "hal/hal_gpio.h"
#include <stdio.h>

/* 记录 PWM 写入历史 */
#define MOCK_PWM_LOG_SIZE 128

typedef struct {
    int pin;
    int duty;
} pwm_log_entry_t;

static pwm_log_entry_t g_pwm_log[MOCK_PWM_LOG_SIZE];
static int g_pwm_log_count = 0;
static int g_gpio_init_called = 0;

int hal_gpio_init(void)
{
    g_gpio_init_called = 1;
    g_pwm_log_count = 0;
    return 0;
}

void hal_gpio_deinit(void) {}

void hal_gpio_set_mode(int pin, int mode) { (void)pin; (void)mode; }
void hal_gpio_write(int pin, int value) { (void)pin; (void)value; }
int  hal_gpio_read(int pin) { (void)pin; return 0; }

int  hal_pwm_set_range(int pin, int range) { (void)pin; (void)range; return 0; }
int  hal_pwm_set_freq(int pin, int freq) { (void)pin; (void)freq; return 0; }

void hal_pwm_write(int pin, int duty)
{
    if (g_pwm_log_count < MOCK_PWM_LOG_SIZE) {
        g_pwm_log[g_pwm_log_count].pin  = pin;
        g_pwm_log[g_pwm_log_count].duty = duty;
        g_pwm_log_count++;
    }
}

void hal_delay_us(unsigned int us) { (void)us; }

/* ---- 测试辅助 ---- */

void mock_reset(void)
{
    g_pwm_log_count = 0;
    g_gpio_init_called = 0;
}

int mock_get_pwm_log(int index, int *pin, int *duty)
{
    if (index < 0 || index >= g_pwm_log_count) return -1;
    if (pin)  *pin  = g_pwm_log[index].pin;
    if (duty) *duty = g_pwm_log[index].duty;
    return 0;
}

int mock_get_pwm_log_count(void) { return g_pwm_log_count; }

int mock_gpio_was_init(void) { return g_gpio_init_called; }
