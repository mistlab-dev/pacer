/**
 * @file app.c
 * @brief 应用层 — 系统初始化、主循环、三种运行模式
 *
 * 三种模式:
 *   1. 正常运行 (APP_MODE_RUN)
 *      → 校准 → 等待用户放正 → 启动平衡 → 200Hz 控制循环
 *
 *   2. 校准模式 (APP_MODE_CALIBRATE)
 *      → 只初始化 IMU, 校准陀螺仪, 输出偏移值
 *
 *   3. 调试模式 (APP_MODE_DEBUG)
 *      → 初始化 IMU, 打印姿态数据, 不驱动电机
 */

#include "app/app.h"
#include "app/config.h"

#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "filter/filter.h"
#include "ctrl/balance.h"
#include "motor/motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

/* ================ 全局状态 ================ */

static volatile int g_running = 1;

static balance_t  g_bal;
static motor_t    g_mot_l;
static motor_t    g_mot_r;

/* ================ 信号 ================ */

static void on_signal(int sig)
{
    printf("\n[APP] signal %d, shutting down...\n", sig);
    g_running = 0;
}

/* ================ 内部: 系统初始化 ================ */

static int sys_init(void)
{
    printf("===== Pacer v1.0 — 二轮平衡车 =====\n");

    /* HAL */
    if (hal_gpio_init() < 0) {
        fprintf(stderr, "[APP] gpio init failed — are you root?\n");
        return -1;
    }

    /* 注册 ICM20948 驱动 */
    imu_icm20948_register();

    /* IMU */
    imu_config_t imu_cfg = IMU_CONFIG_DEFAULT;
    if (imu_init(&imu_cfg) != 0) {
        fprintf(stderr, "[APP] IMU init failed\n");
        return -2;
    }

    /* 姿态滤波器 */
    filter_config_t f_cfg = FILTER_CONFIG_DEFAULT;
#if !CFG_USE_MADGWICK
    f_cfg.type = FILTER_COMPLEMENTARY;
#endif
    filter_init(&f_cfg);

    /* 电机 */
    motor_init(&g_mot_l, CFG_MOTOR_LEFT_A,  CFG_MOTOR_LEFT_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_LEFT_INV);
    motor_init(&g_mot_r, CFG_MOTOR_RIGHT_A, CFG_MOTOR_RIGHT_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_RIGHT_INV);

    /* 平衡控制器 */
    balance_init(&g_bal);

    printf("[APP] all systems ready\n");
    return 0;
}

static void sys_deinit(void)
{
    printf("[APP] cleaning up...\n");
    balance_enable(&g_bal, false);
    motor_deinit(&g_mot_l);
    motor_deinit(&g_mot_r);
    imu_deinit();
    hal_gpio_deinit();
    printf("[APP] bye\n");
}

/* ================ 模式: 校准 ================ */

static int mode_calibrate(void)
{
    if (hal_gpio_init() < 0) {
        fprintf(stderr, "gpio init failed\n");
        return -1;
    }

    imu_icm20948_register();

    imu_config_t cfg = IMU_CONFIG_DEFAULT;
    if (imu_init(&cfg) != 0) {
        hal_gpio_deinit();
        return -2;
    }

    printf("\n=== IMU 校准 ===\n");
    printf("请保持传感器静止, 按 Enter 开始...\n");
    getchar();

    imu_calibrate_gyro(500);

    vec3_t bias;
    imu_get_gyro_bias(&bias);
    printf("\n校准结果 (°/s):\n");
    printf("  X = %.4f\n", bias.x);
    printf("  Y = %.4f\n", bias.y);
    printf("  Z = %.4f\n", bias.z);

    imu_deinit();
    hal_gpio_deinit();
    return 0;
}

/* ================ 模式: 调试 ================ */

static int mode_debug(void)
{
    if (hal_gpio_init() < 0) return -1;

    imu_icm20948_register();

    imu_config_t cfg = IMU_CONFIG_DEFAULT;
    if (imu_init(&cfg) != 0) {
        hal_gpio_deinit();
        return -2;
    }

    filter_config_t f_cfg = FILTER_CONFIG_DEFAULT;
    filter_init(&f_cfg);

    printf("\n=== 调试模式 (Ctrl+C 退出) ===\n\n");
    printf("%-8s %-8s %-8s  | %-8s %-8s %-8s\n",
           "Roll", "Pitch", "Yaw", "aX", "aY", "aZ");
    printf("─────────────────────────────────────────────\n");

    imu_sample_t sample;
    while (g_running) {
        if (imu_read(&sample) == 0) {
            attitude_t att = filter_update(&sample, CFG_CONTROL_DT);
            printf("%-8.2f %-8.2f %-8.2f  | %-8.2f %-8.2f %-8.2f\n",
                   att.roll, att.pitch, att.yaw,
                   sample.accel.x, sample.accel.y, sample.accel.z);
        }
        usleep(50000);  /* 20Hz 显示 */
    }

    imu_deinit();
    hal_gpio_deinit();
    return 0;
}

/* ================ 模式: 正常运行 ================ */

static int mode_run(void)
{
    if (sys_init() != 0) return -1;

    /* 校准 */
    printf("\n[APP] 自动校准中... 请保持静止\n");
    imu_calibrate_gyro(200);
    usleep(100000);

    /* 等用户就位 */
    printf("[APP] 扶正车体, 按 Enter 启动平衡控制...\n");
    getchar();

    /* 启动! */
    balance_enable(&g_bal, true);
    printf("[APP] 平衡控制已启动! Ctrl+C 停止\n\n");

    /* === 主控制循环 === */
    imu_sample_t sample;
    int tick = 0;

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (g_running) {
        /* 1. 读 IMU */
        if (imu_read(&sample) != 0) {
            usleep(1000);
            continue;
        }

        /* 2. 姿态解算 */
        attitude_t att = filter_update(&sample, CFG_CONTROL_DT);

        /* 3. 平衡控制 */
        const balance_output_t *out = balance_update(&g_bal, &att, &sample,
                                                     CFG_CONTROL_DT);

        /* 4. 输出到电机 */
        if (!out->emergency) {
            motor_set(&g_mot_l, out->motor_left);
            motor_set(&g_mot_r, out->motor_right);
        }

        /* 5. 控制台日志 (10Hz) */
#if CFG_ENABLE_CONSOLE_LOG
        if (tick % 20 == 0) {
            const balance_debug_t *d = balance_get_debug(&g_bal);
            printf("roll=%+7.2f°  target=%+6.2f°  L=%+5.0f R=%+5.0f\n",
                   d->roll, d->roll_target,
                   out->motor_left, out->motor_right);
        }
#endif

        tick++;

        /* 6. 精确延时 — 保持 200Hz */
        struct timespec t_next = t0;
        long add_ns = (tick) * (long)CFG_CONTROL_INTERVAL_US * 1000L;
        t_next.tv_sec  += add_ns / 1000000000L;
        t_next.tv_nsec += add_ns % 1000000000L;
        if (t_next.tv_nsec >= 1000000000L) {
            t_next.tv_sec  += 1;
            t_next.tv_nsec -= 1000000000L;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long wait_ns = (t_next.tv_sec - now.tv_sec) * 1000000000L
                     + (t_next.tv_nsec - now.tv_nsec);
        if (wait_ns > 0) {
            usleep((useconds_t)(wait_ns / 1000));
        }
    }

    sys_deinit();
    return 0;
}

/* ================ 入口 ================ */

int app_run(app_mode_t mode)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    switch (mode) {
        case APP_MODE_CALIBRATE: return mode_calibrate();
        case APP_MODE_DEBUG:     return mode_debug();
        case APP_MODE_RUN:       return mode_run();
    }
    return 1;
}
