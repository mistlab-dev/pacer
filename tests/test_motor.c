/**
 * @file test_motor.c
 * @brief 电机驱动单元测试 (DC GPIO + ESC 模式)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor/motor.h"
#include "hal/hal_pca9685.h"
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

/* ============ DC GPIO 测试 ============ */

static void test_dc_forward(void)
{
    printf("[test] DC forward\n");
    mock_reset();
    motor_t m;
    motor_init_dc(&m, MOTOR_TYPE_DC_GPIO, 12, 13, 1000, false);
    mock_reset();
    motor_set(&m, 500.0f);

    int a, b;
    ASSERT(find_pwm_for_pin(12, &a) == 0, "pin_a");
    ASSERT(find_pwm_for_pin(13, &b) == 0, "pin_b");
    ASSERT_F(a == 500, "duty_a = %d", a);
    ASSERT_F(b == 0,   "duty_b = %d", b);
}

static void test_dc_reverse(void)
{
    printf("[test] DC reverse\n");
    mock_reset();
    motor_t m;
    motor_init_dc(&m, MOTOR_TYPE_DC_GPIO, 12, 13, 1000, false);
    mock_reset();
    motor_set(&m, -300.0f);

    int a, b;
    ASSERT(find_pwm_for_pin(12, &a) == 0, "pin_a");
    ASSERT(find_pwm_for_pin(13, &b) == 0, "pin_b");
    ASSERT_F(a == 0,   "duty_a = %d", a);
    ASSERT_F(b == 300, "duty_b = %d", b);
}

static void test_dc_inverted(void)
{
    printf("[test] DC inverted\n");
    mock_reset();
    motor_t m;
    motor_init_dc(&m, MOTOR_TYPE_DC_GPIO, 12, 13, 1000, true);
    mock_reset();
    motor_set(&m, 500.0f);

    int b;
    ASSERT(find_pwm_for_pin(13, &b) == 0, "pin_b");
    ASSERT_F(b == 500, "duty_b = %d", b);
}

static void test_dc_brake(void)
{
    printf("[test] DC brake\n");
    mock_reset();
    motor_t m;
    motor_init_dc(&m, MOTOR_TYPE_DC_GPIO, 12, 13, 1000, false);
    mock_reset();
    motor_brake(&m);

    int a, b;
    ASSERT(find_pwm_for_pin(12, &a) == 0, "pin_a");
    ASSERT(find_pwm_for_pin(13, &b) == 0, "pin_b");
    ASSERT_F(a == 1000, "brake_a = %d", a);
    ASSERT_F(b == 1000, "brake_b = %d", b);
}

/* ---- ESC 测试辅助 ---- */

static void esc_mock_init(void)
{
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 50);  /* mock PCA9685, open=true */
}

#define ESC_1000US  205
#define ESC_1500US  307
#define ESC_2000US  410
#define ESC_TOLERANCE 3

static void test_esc_init(void)
{
    printf("[test] ESC init (中位信号)\n");
    esc_mock_init();
    mock_i2c_reset();  /* 清除 pca9685_init 的写入, 但保留 dev.open */
    motor_t m;
    motor_init_esc(&m, 0, false);

    /* init 时应输出中位 1500μs */
    /* CH0 base = 0x06 */
    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_1500US) <= ESC_TOLERANCE,
             "init off = %d, expected ~%d", off, ESC_1500US);
    ASSERT(m.type == MOTOR_TYPE_ESC, "type is ESC");
    ASSERT(m.armed == false, "not armed yet");
}

static void test_esc_arm(void)
{
    printf("[test] ESC arm\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, false);

    motor_esc_arm(&m);
    ASSERT(m.armed == true, "should be armed");

    /* arm 输出中位 1500μs */
    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_1500US) <= ESC_TOLERANCE,
             "arm off = %d, expected ~%d", off, ESC_1500US);
}

static void test_esc_full_throttle(void)
{
    printf("[test] ESC full throttle (+1.0)\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, false);
    motor_esc_arm(&m);
    mock_i2c_reset();

    motor_set(&m, 1.0f);

    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_2000US) <= ESC_TOLERANCE,
             "full throttle off = %d, expected ~%d", off, ESC_2000US);
}

static void test_esc_full_reverse(void)
{
    printf("[test] ESC full reverse (-1.0)\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, false);
    motor_esc_arm(&m);
    mock_i2c_reset();

    motor_set(&m, -1.0f);

    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_1000US) <= ESC_TOLERANCE,
             "full reverse off = %d, expected ~%d", off, ESC_1000US);
}

static void test_esc_stop(void)
{
    printf("[test] ESC stop (0.0)\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, false);
    motor_esc_arm(&m);
    mock_i2c_reset();

    motor_set(&m, 0.0f);

    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_1500US) <= ESC_TOLERANCE,
             "stop off = %d, expected ~%d", off, ESC_1500US);
}

static void test_esc_not_armed(void)
{
    printf("[test] ESC not armed — motor_set ignored\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, false);
    /* 不 arm */

    mock_i2c_reset();
    motor_set(&m, 1.0f);

    /* 未 armed, motor_set 不应写入 (寄存器保持 0) */
    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(off == 0 || abs(off - ESC_1500US) <= ESC_TOLERANCE,
             "unarmed off = %d (should be 0 or mid)", off);
}

static void test_esc_inverted(void)
{
    printf("[test] ESC inverted (+1.0 → 1000μs)\n");
    esc_mock_init();
    motor_t m;
    motor_init_esc(&m, 0, true);
    motor_esc_arm(&m);
    mock_i2c_reset();

    motor_set(&m, 1.0f);  /* inverted → 实际 -1.0 → 1000μs */

    uint8_t base = 0x06;
    uint16_t off = (uint16_t)(mock_i2c_get_reg(base + 3) << 8 | mock_i2c_get_reg(base + 2));
    ASSERT_F(abs(off - ESC_1000US) <= ESC_TOLERANCE,
             "inverted off = %d, expected ~%d", off, ESC_1000US);
}

int main(void)
{
    printf("=== Motor Tests (DC + ESC) ===\n\n");

    hal_gpio_init();

    test_dc_forward();
    test_dc_reverse();
    test_dc_inverted();
    test_dc_brake();

    test_esc_init();
    test_esc_arm();
    test_esc_full_throttle();
    test_esc_full_reverse();
    test_esc_stop();
    test_esc_not_armed();
    test_esc_inverted();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
