/**
 * @file test_motor.c
 * @brief 电机驱动单元测试 (GPIO PWM 模式)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor/motor.h"
#include "app/config.h"
#include "mock_helpers.h"

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL: %s\n", msg); } \
} while(0)

#define ASSERT_F(cond, fmt, ...) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  FAIL: " fmt "\n", __VA_ARGS__); } \
} while(0)

/**
 * 验证最近一次 PWM 写入
 * motor_set 会写 pin_a 和 pin_b 两个通道
 */
static int find_pwm_for_pin(int pin, int *duty)
{
    for (int i = mock_get_pwm_log_count() - 1; i >= 0; i--) {
        int p, d;
        if (mock_get_pwm_log(i, &p, &d) == 0 && p == pin) {
            if (duty) *duty = d;
            return 0;
        }
    }
    return -1;
}

static void test_motor_forward(void)
{
    printf("[test] motor forward\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, false);

    mock_reset();  /* 清除 init 的写入 */
    motor_set(&m, 500.0f);

    int duty_a, duty_b;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a should be written");
    ASSERT(find_pwm_for_pin(13, &duty_b) == 0, "pin_b should be written");
    ASSERT_F(duty_a == 500, "pin_a duty = %d, expected 500", duty_a);
    ASSERT_F(duty_b == 0,   "pin_b duty = %d, expected 0", duty_b);
}

static void test_motor_reverse(void)
{
    printf("[test] motor reverse\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, false);

    mock_reset();
    motor_set(&m, -300.0f);

    int duty_a, duty_b;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a written");
    ASSERT(find_pwm_for_pin(13, &duty_b) == 0, "pin_b written");
    ASSERT_F(duty_a == 0,   "pin_a duty = %d, expected 0", duty_a);
    ASSERT_F(duty_b == 300, "pin_b duty = %d, expected 300", duty_b);
}

static void test_motor_inverted(void)
{
    printf("[test] motor inverted\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, true);

    mock_reset();
    motor_set(&m, 500.0f);  /* inverted, 所以实际反转 */

    int duty_a, duty_b;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a written");
    ASSERT(find_pwm_for_pin(13, &duty_b) == 0, "pin_b written");
    ASSERT_F(duty_a == 0,   "pin_a duty = %d, expected 0", duty_a);
    ASSERT_F(duty_b == 500, "pin_b duty = %d, expected 500", duty_b);
}

static void test_motor_stop(void)
{
    printf("[test] motor stop (power=0)\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, false);

    mock_reset();
    motor_set(&m, 0.0f);

    int duty_a, duty_b;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a written");
    ASSERT(find_pwm_for_pin(13, &duty_b) == 0, "pin_b written");
    ASSERT_F(duty_a == 0, "pin_a duty = %d", duty_a);
    ASSERT_F(duty_b == 0, "pin_b duty = %d", duty_b);
}

static void test_motor_brake(void)
{
    printf("[test] motor brake\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, false);

    mock_reset();
    motor_brake(&m);

    int duty_a, duty_b;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a written");
    ASSERT(find_pwm_for_pin(13, &duty_b) == 0, "pin_b written");
    ASSERT_F(duty_a == 1000, "pin_a brake duty = %d", duty_a);
    ASSERT_F(duty_b == 1000, "pin_b brake duty = %d", duty_b);
}

static void test_motor_clamp(void)
{
    printf("[test] motor output clamp\n");
    mock_reset();
    motor_t m;
    motor_init(&m, MOTOR_PWM_GPIO, 12, 13, 1000, false);

    mock_reset();
    motor_set(&m, 9999.0f);  /* 超过 pwm_range */

    int duty_a;
    ASSERT(find_pwm_for_pin(12, &duty_a) == 0, "pin_a written");
    ASSERT_F(duty_a == 1000, "clamped duty = %d, expected 1000", duty_a);
}

int main(void)
{
    printf("=== Motor Tests ===\n\n");

    hal_gpio_init();

    test_motor_forward();
    test_motor_reverse();
    test_motor_inverted();
    test_motor_stop();
    test_motor_brake();
    test_motor_clamp();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
