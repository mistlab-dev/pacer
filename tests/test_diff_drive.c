/**
 * @file test_diff_drive.c
 * @brief 差速驱动单元测试 (DC + ESC 模式)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor/motor.h"
#include "hal/hal_pca9685.h"
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

/* ---- DC 模式测试 ---- */

static void setup_dc(diff_drive_t *dd)
{
    static motor_t motors[4];
    motor_init_dc(&motors[MOTOR_FL], MOTOR_TYPE_DC_GPIO, 0, 1, 1000, false);
    motor_init_dc(&motors[MOTOR_FR], MOTOR_TYPE_DC_GPIO, 2, 3, 1000, true);
    motor_init_dc(&motors[MOTOR_RL], MOTOR_TYPE_DC_GPIO, 4, 5, 1000, false);
    motor_init_dc(&motors[MOTOR_RR], MOTOR_TYPE_DC_GPIO, 6, 7, 1000, true);
    diff_drive_init(dd, motors);
}

static void test_dc_forward(void)
{
    printf("[test] DC diff_drive forward\n");
    mock_reset();
    diff_drive_t dd;
    setup_dc(&dd);
    diff_drive_enable(&dd, true);
    mock_reset();

    diff_drive_set(&dd, 0.5f, 0.0f);
    diff_drive_update(&dd, 0.01f);

    /* left=500, FL not inverted → pin_a=0 (duty_a), pin_b=1 (duty_b=0) */
    /* right=500, FR inverted → actual=-500 → pin_a=2(duty=0), pin_b=3(duty=500) */
    int fr_b = -1;
    for (int i = mock_get_pwm_log_count() - 1; i >= 0; i--) {
        int p, d;
        mock_get_pwm_log(i, &p, &d);
        if (p == 3) { fr_b = d; break; }
    }
    ASSERT_F(fr_b == 500, "FR pin_b = %d", fr_b);
}

static void test_dc_disabled(void)
{
    printf("[test] DC diff_drive disabled — outputs zero\n");
    mock_reset();
    diff_drive_t dd;
    setup_dc(&dd);
    /* 不 enable */

    mock_reset();
    diff_drive_set(&dd, 1.0f, 0.0f);
    diff_drive_update(&dd, 0.01f);

    /* disabled 时 motor_set(0) 也会写 GPIO, 检查写入都是 0 */
    int d;
    if (mock_get_pwm_log(0, NULL, &d) == 0) {
        ASSERT_F(d == 0, "disabled output duty = %d (should be 0)", d);
    } else {
        g_pass++; /* 没有 PWM 写入也 OK */
    }
}

/* ---- ESC 模式测试 ---- */

static void esc_mock_init(void)
{
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 50);
}

static void setup_esc(diff_drive_t *dd)
{
    esc_mock_init();
    mock_i2c_reset();
    static motor_t motors[4];
    motor_init_esc(&motors[MOTOR_FL], 0, false);
    motor_init_esc(&motors[MOTOR_FR], 1, true);
    motor_init_esc(&motors[MOTOR_RL], 2, false);
    motor_init_esc(&motors[MOTOR_RR], 3, true);

    /* arm 所有 ESC */
    for (int i = 0; i < 4; i++) motor_esc_arm(&motors[i]);

    diff_drive_init(dd, motors);
}

#define ESC_1500US  307
#define ESC_2000US  410
#define ESC_TOL     3

static uint16_t get_ch_off(int ch)
{
    uint8_t base = 0x06 + 4 * ch;
    return (uint16_t)((mock_i2c_get_reg(base + 3) << 8) | mock_i2c_get_reg(base + 2));
}

static void test_esc_forward(void)
{
    printf("[test] ESC diff_drive forward\n");
    mock_i2c_reset();
    diff_drive_t dd;
    setup_esc(&dd);
    diff_drive_enable(&dd, true);
    mock_i2c_reset();

    diff_drive_set(&dd, 0.5f, 0.0f);
    diff_drive_update(&dd, 0.01f);

    /* left = 0.5 → 1750μs */
    uint16_t fl = get_ch_off(0);
    /* right = 0.5, inverted → -0.5 → 1250μs */
    uint16_t fr = get_ch_off(1);

    /* 0.5: 1500 + 0.5*500 = 1750μs → 1750*4096/20000 ≈ 358 ticks */
    int expected_l = (int)(1750.0f * 4096.0f / 20000.0f);
    /* -0.5: 1500 - 250 = 1250μs → 1250*4096/20000 ≈ 256 ticks */
    int expected_r = (int)(1250.0f * 4096.0f / 20000.0f);

    ASSERT_F(abs((int)fl - expected_l) <= ESC_TOL,
             "FL off = %d, expected ~%d", fl, expected_l);
    ASSERT_F(abs((int)fr - expected_r) <= ESC_TOL,
             "FR off = %d, expected ~%d", fr, expected_r);
}

static void test_esc_spin(void)
{
    printf("[test] ESC diff_drive spin right\n");
    mock_i2c_reset();
    diff_drive_t dd;
    setup_esc(&dd);
    diff_drive_enable(&dd, true);
    mock_i2c_reset();

    diff_drive_set(&dd, 0.0f, 1.0f);  /* 纯右转 */
    diff_drive_update(&dd, 0.01f);

    /* left = 0-1 = -1.0 → 1000μs, right = 0+1 = 1.0 inverted → -1.0 → 1000μs */
    uint16_t fl = get_ch_off(0);
    uint16_t fr = get_ch_off(1);

    int expected_1000 = (int)(1000.0f * 4096.0f / 20000.0f);  /* 205 */

    ASSERT_F(abs((int)fl - expected_1000) <= ESC_TOL,
             "FL spin off = %d, expected ~%d", fl, expected_1000);
    ASSERT_F(abs((int)fr - expected_1000) <= ESC_TOL,
             "FR spin off = %d, expected ~%d", fr, expected_1000);
}

static void test_esc_stop(void)
{
    printf("[test] ESC diff_drive stop\n");
    mock_i2c_reset();
    diff_drive_t dd;
    setup_esc(&dd);
    diff_drive_enable(&dd, true);
    mock_i2c_reset();

    diff_drive_set(&dd, 0.0f, 0.0f);
    diff_drive_update(&dd, 0.01f);

    for (int ch = 0; ch < 4; ch++) {
        uint16_t off = get_ch_off(ch);
        ASSERT_F(abs((int)off - ESC_1500US) <= ESC_TOL,
                 "CH%d stop off = %d, expected ~%d", ch, off, ESC_1500US);
    }
}

static void test_dd_steering_clamp(void)
{
    printf("[test] diff_drive steering clamp\n");
    diff_drive_t dd;
    setup_esc(&dd);

    diff_drive_set(&dd, 0.0f, 5.0f);
    ASSERT_F(fabsf(dd.steering - 1.0f) < 0.001f, "steering = %.2f", dd.steering);
}

int main(void)
{
    printf("=== Diff Drive Tests (DC + ESC) ===\n\n");

    hal_gpio_init();

    test_dc_forward();
    test_dc_disabled();
    test_esc_forward();
    test_esc_spin();
    test_esc_stop();
    test_dd_steering_clamp();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
