/**
 * @file test_pid.c
 * @brief PID 控制器单元测试
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ctrl/pid.h"

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

static void test_pid_init(void)
{
    printf("[test] pid_init\n");
    pacer_pid_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.01f, 10.0f, 100.0f);

    ASSERT_F(pid.kp == 1.0f, "kp = %.2f", pid.kp);
    ASSERT_F(pid.ki == 0.1f, "ki = %.2f", pid.ki);
    ASSERT_F(pid.kd == 0.01f, "kd = %.2f", pid.kd);
    ASSERT_F(pid.integral_limit == 10.0f, "integral_limit = %.2f", pid.integral_limit);
    ASSERT_F(pid.output_limit == 100.0f, "output_limit = %.2f", pid.output_limit);
    ASSERT_F(pid.integral == 0.0f, "integral = %.2f", pid.integral);
}

static void test_pid_pure_p(void)
{
    printf("[test] pid pure proportional\n");
    pacer_pid_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    float out = pid_compute(&pid, 5.0f, 0.01f);
    ASSERT_F(fabsf(out - 10.0f) < 0.001f, "out = %.3f, expected 10.0", out);
}

static void test_pid_integral(void)
{
    printf("[test] pid integral accumulates\n");
    pacer_pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 100.0f, 0.0f);

    float dt = 0.01f;
    pid_compute(&pid, 10.0f, dt);  /* integral = 10 * 0.01 = 0.1 */
    pid_compute(&pid, 10.0f, dt);  /* integral = 0.2 */

    ASSERT_F(fabsf(pid.integral - 0.2f) < 0.001f, "integral = %.4f", pid.integral);
}

static void test_pid_integral_clamp(void)
{
    printf("[test] pid integral clamping\n");
    pacer_pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f);  /* integral_limit=1.0 */

    /* 多次计算让积分饱和 */
    for (int i = 0; i < 200; i++) {
        pid_compute(&pid, 10.0f, 0.01f);
    }

    ASSERT_F(fabsf(pid.integral - 1.0f) < 0.001f,
             "integral clamped = %.4f", pid.integral);
}

static void test_pid_derivative(void)
{
    printf("[test] pid derivative\n");
    pacer_pid_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);

    float dt = 0.01f;
    float out = pid_compute(&pid, 5.0f, dt);  /* d = (5-0)/0.01 = 500 */
    ASSERT_F(fabsf(out - 500.0f) < 0.1f, "d out = %.2f", out);

    float out2 = pid_compute(&pid, 5.0f, dt);  /* d = (5-5)/0.01 = 0 */
    ASSERT_F(fabsf(out2) < 0.001f, "d steady = %.4f", out2);
}

static void test_pid_output_limit(void)
{
    printf("[test] pid output limit\n");
    pacer_pid_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.0f, 50.0f);  /* output_limit=50 */

    float out = pid_compute(&pid, 1.0f, 0.01f);
    /* P = 100 * 1 = 100, clamped to 50 */
    ASSERT_F(fabsf(out - 50.0f) < 0.001f, "clamped out = %.2f", out);
}

static void test_pid_reset(void)
{
    printf("[test] pid reset\n");
    pacer_pid_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 10.0f, 100.0f);

    pid_compute(&pid, 5.0f, 0.01f);
    pid_compute(&pid, 3.0f, 0.01f);
    ASSERT(pid.integral != 0.0f, "integral should be non-zero");

    pid_reset(&pid);
    ASSERT_F(fabsf(pid.integral) < 0.001f, "integral after reset = %.4f", pid.integral);
    ASSERT_F(fabsf(pid.prev_error) < 0.001f, "prev_error after reset = %.4f", pid.prev_error);
}

static void test_pid_set_gains(void)
{
    printf("[test] pid set_gains\n");
    pacer_pid_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pid_compute(&pid, 5.0f, 0.01f);  /* integral = 0.05 */

    pid_set_gains(&pid, 2.0f, 0.5f, 0.1f);
    ASSERT_F(pid.kp == 2.0f, "kp = %.2f", pid.kp);
    /* 状态不应被清除 */
    ASSERT_F(fabsf(pid.integral) > 0.001f, "integral preserved = %.4f", pid.integral);
}

int main(void)
{
    printf("=== PID Tests ===\n\n");

    test_pid_init();
    test_pid_pure_p();
    test_pid_integral();
    test_pid_integral_clamp();
    test_pid_derivative();
    test_pid_output_limit();
    test_pid_reset();
    test_pid_set_gains();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
