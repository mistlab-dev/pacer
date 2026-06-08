/**
 * @file test_diff_drive.c
 * @brief 差速驱动控制器单元测试
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor/motor.h"
#include "ctrl/diff_drive.h"
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

/* 记录每个电机 pin 最终的 PWM duty */
static int get_last_duty_for_pin(int pin)
{
    for (int i = mock_get_pwm_log_count() - 1; i >= 0; i--) {
        int p, d;
        if (mock_get_pwm_log(i, &p, &d) == 0 && p == pin)
            return d;
    }
    return -1;
}

/* 获取电机 pin_a / pin_b 的最终 duty */
static void get_motor_duties(const motor_t *m, int *a, int *b)
{
    *a = get_last_duty_for_pin(m->pin_a);
    *b = get_last_duty_for_pin(m->pin_b);
}

static void setup_4wd(diff_drive_t *dd)
{
    /* 4 个 GPIO PWM 电机 (测试用 mock) */
    static motor_t motors[4];
    motor_init(&motors[MOTOR_FL], MOTOR_PWM_GPIO, 0, 1, 1000, false);
    motor_init(&motors[MOTOR_FR], MOTOR_PWM_GPIO, 2, 3, 1000, true);
    motor_init(&motors[MOTOR_RL], MOTOR_PWM_GPIO, 4, 5, 1000, false);
    motor_init(&motors[MOTOR_RR], MOTOR_PWM_GPIO, 6, 7, 1000, true);

    diff_drive_init(dd, motors);
}

static void test_dd_straight_forward(void)
{
    printf("[test] diff_drive straight forward\n");
    mock_reset();
    diff_drive_t dd;
    setup_4wd(&dd);
    diff_drive_enable(&dd, true);

    mock_reset();
    diff_drive_set(&dd, 0.5f, 0.0f);  /* 半速前进 */
    diff_drive_update(&dd, 0.01f);

    /* 左侧: 0.5 * 1000 = 500 (pin_a=500, pin_b=0) */
    /* 右侧: inverted, 所以 0.5 → reverse → pin_a=0, pin_b=500 */
    int fl_a, fl_b, fr_a, fr_b, rl_a, rl_b, rr_a, rr_b;
    get_motor_duties(&dd.motors[MOTOR_FL], &fl_a, &fl_b);
    get_motor_duties(&dd.motors[MOTOR_FR], &fr_a, &fr_b);
    get_motor_duties(&dd.motors[MOTOR_RL], &rl_a, &rl_b);
    get_motor_duties(&dd.motors[MOTOR_RR], &rr_a, &rr_b);

    /* 前左 + 后左: 正转 500 */
    ASSERT_F(fl_a == 500, "FL pin_a = %d", fl_a);
    ASSERT_F(fl_b == 0,   "FL pin_b = %d", fl_b);
    ASSERT_F(rl_a == 500, "RL pin_a = %d", rl_a);

    /* 前右 + 后右: inverted, throttle=0.5 → power=-500 → pin_b=500 */
    ASSERT_F(fr_b == 500, "FR pin_b = %d", fr_b);
    ASSERT_F(rr_b == 500, "RR pin_b = %d", rr_b);
}

static void test_dd_spin_right(void)
{
    printf("[test] diff_drive spin right (pivot)\n");
    mock_reset();
    diff_drive_t dd;
    setup_4wd(&dd);
    diff_drive_enable(&dd, true);

    mock_reset();
    diff_drive_set(&dd, 0.0f, 0.5f);  /* throttle=0, steer=0.5 右转 */
    diff_drive_update(&dd, 0.01f);

    /* left  = (0 - 0.5) * 1000 = -500 → 反转 */
    /* right = (0 + 0.5) * 1000 = 500 → but inverted → 反转 */
    /* 所以左侧反转，右侧反转(因为inverted) → 实际原地右转 */

    int fl_b = get_last_duty_for_pin(1);  /* FL 反转 */
    int fr_a = get_last_duty_for_pin(2);  /* FR inverted, positive power → pin_a=0 */
    int fr_b = get_last_duty_for_pin(3);  /* FR inverted, negative → pin_b */

    /* 左侧 left=-500, FL not inverted → 反转: pin_a=0, pin_b=500 */
    ASSERT_F(fl_b == 500, "FL reverse duty = %d", fl_b);
    /* 右侧 right=500, FR inverted → actual -500 → 反转: pin_b=500 */
    ASSERT_F(fr_b == 500, "FR inverted reverse duty = %d", fr_b);
}

static void test_dd_disabled(void)
{
    printf("[test] diff_drive disabled outputs zero\n");
    mock_reset();
    diff_drive_t dd;
    setup_4wd(&dd);
    /* 不 enable */

    mock_reset();
    diff_drive_set(&dd, 1.0f, 1.0f);
    diff_drive_update(&dd, 0.01f);

    int fl_a = get_last_duty_for_pin(0);
    int fl_b = get_last_duty_for_pin(1);
    ASSERT_F(fl_a == 0, "FL pin_a = %d (should be 0)", fl_a);
    ASSERT_F(fl_b == 0, "FL pin_b = %d (should be 0)", fl_b);
}

static void test_dd_brake(void)
{
    printf("[test] diff_drive brake\n");
    mock_reset();
    diff_drive_t dd;
    setup_4wd(&dd);
    diff_drive_enable(&dd, true);

    mock_reset();
    diff_drive_brake(&dd);

    /* 刹车: 所有 pin 都拉满 */
    for (int i = 0; i < 8; i++) {
        int d = get_last_duty_for_pin(i);
        ASSERT_F(d == 1000, "pin %d brake duty = %d", i, d);
    }
}

static void test_dd_clamp_steering(void)
{
    printf("[test] diff_drive steering clamp\n");
    mock_reset();
    diff_drive_t dd;
    setup_4wd(&dd);
    diff_drive_enable(&dd, true);

    mock_reset();
    diff_drive_set(&dd, 0.0f, 5.0f);  /* 超出范围，应 clamp 到 1.0 */
    ASSERT_F(fabsf(dd.steering - 1.0f) < 0.001f, "steering = %.2f", dd.steering);
}

int main(void)
{
    printf("=== Diff Drive Tests ===\n\n");

    hal_gpio_init();

    test_dd_straight_forward();
    test_dd_spin_right();
    test_dd_disabled();
    test_dd_brake();
    test_dd_clamp_steering();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
