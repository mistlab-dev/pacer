/**
 * @file test_pca9685.c
 * @brief PCA9685 驱动单元测试
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "hal/hal_pca9685.h"
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

/* PCA9685 寄存器 */
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_PRESCALE    0xFE
#define REG_LED0_ON_L   0x06

static void test_pca9685_init(void)
{
    printf("[test] pca9685 init\n");
    mock_i2c_reset();

    int ret = hal_pca9685_init(1, 0x40, 1000);
    ASSERT_F(ret == 0, "init returned %d", ret);
    ASSERT(mock_i2c_was_opened(), "i2c_open should be called");

    /* prescale = 25000000 / (4096 * 1000) - 1 ≈ 5 */
    uint8_t prescale = mock_i2c_get_reg(REG_PRESCALE);
    ASSERT_F(prescale >= 3, "prescale = %d", prescale);

    /* MODE2 应该被设为 0x04 (推挽) */
    ASSERT_F(mock_i2c_get_reg(REG_MODE2) == 0x04,
             "MODE2 = 0x%02X", mock_i2c_get_reg(REG_MODE2));
}

static void test_pca9685_set_pwm(void)
{
    printf("[test] pca9685 set_pwm channel 0\n");
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 1000);
    mock_i2c_reset();  /* 清除 init 写入 */

    /* duty=500, range=1000 → off = 500 * 4095 / 1000 = 2047 */
    hal_pca9685_set_pwm(0, 500, 1000);

    /* CH0: ON=0 (reg 0x06, 0x07), OFF=2047 (reg 0x08, 0x09) */
    uint8_t on_l  = mock_i2c_get_reg(REG_LED0_ON_L + 0);  /* 0x06 */
    uint8_t on_h  = mock_i2c_get_reg(REG_LED0_ON_L + 1);  /* 0x07 */
    uint8_t off_l = mock_i2c_get_reg(REG_LED0_ON_L + 2);  /* 0x08 */
    uint8_t off_h = mock_i2c_get_reg(REG_LED0_ON_L + 3);  /* 0x09 */

    ASSERT_F(on_l == 0 && on_h == 0, "ON = %d,%d", on_l, on_h);

    uint16_t off = (uint16_t)((off_h << 8) | off_l);
    uint16_t expected = (uint16_t)(500 * 4095 / 1000);
    ASSERT_F(off == expected, "OFF = %d, expected %d", off, expected);
}

static void test_pca9685_set_pwm_full(void)
{
    printf("[test] pca9685 set_pwm full duty\n");
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 1000);
    mock_i2c_reset();

    /* duty=1000, range=1000 → off = 4095 */
    hal_pca9685_set_pwm(2, 1000, 1000);

    /* CH2: base = REG_LED0_ON_L + 4*2 = 0x0E */
    uint8_t base = REG_LED0_ON_L + 4 * 2;
    uint8_t off_l = mock_i2c_get_reg(base + 2);
    uint8_t off_h = mock_i2c_get_reg(base + 3);
    uint16_t off = (uint16_t)((off_h << 8) | off_l);

    ASSERT_F(off == 4095, "full duty OFF = %d", off);
}

static void test_pca9685_set_pwm_zero(void)
{
    printf("[test] pca9685 set_pwm zero duty\n");
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 1000);
    mock_i2c_reset();

    hal_pca9685_set_pwm(0, 0, 1000);

    uint8_t off_l = mock_i2c_get_reg(REG_LED0_ON_L + 2);
    uint8_t off_h = mock_i2c_get_reg(REG_LED0_ON_L + 3);
    ASSERT_F(off_l == 0 && off_h == 0, "zero OFF = %d,%d", off_l, off_h);
}

static void test_pca9685_deinit(void)
{
    printf("[test] pca9685 deinit\n");
    mock_i2c_reset();
    hal_pca9685_init(1, 0x40, 1000);

    hal_pca9685_deinit();
    /* 应该进入 sleep (MODE1 的 sleep 位置 1) */
    uint8_t mode1 = mock_i2c_get_reg(REG_MODE1);
    ASSERT_F(mode1 & 0x10, "MODE1 sleep bit not set: 0x%02X", mode1);
}

int main(void)
{
    printf("=== PCA9685 Tests ===\n\n");

    test_pca9685_init();
    test_pca9685_set_pwm();
    test_pca9685_set_pwm_full();
    test_pca9685_set_pwm_zero();
    test_pca9685_deinit();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
