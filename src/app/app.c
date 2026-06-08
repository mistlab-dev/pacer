/**
 * @file app.c
 * @brief 应用层 — 四轮差速驱动
 *
 * 运行模式:
 *   1. 正常运行 (APP_MODE_RUN)
 *      → 初始化 PCA9685 + 4电机 → 差速驱动控制循环
 *
 *   2. 校准模式 (APP_MODE_CALIBRATE)
 *      → 只初始化 IMU, 校准陀螺仪
 *
 *   3. 调试模式 (APP_MODE_DEBUG)
 *      → 打印姿态数据, 不驱动电机
 */

#include "app/app.h"
#include "app/config.h"

#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_pca9685.h"
#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "filter/filter.h"
#include "ctrl/diff_drive.h"
#include "motor/motor.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

/* ================ 全局状态 ================ */

static volatile int g_running = 1;

static diff_drive_t g_dd;
static motor_t g_motors[DIFF_DRIVE_NUM_MOTORS];

/* ================ 信号 ================ */

static void on_signal(int sig)
{
    printf("\n[APP] signal %d, shutting down...\n", sig);
    g_running = 0;
}

/* ================ 构建4个电机 ================ */

#if CFG_MOTOR_PWM_SRC == 1
#define MOTOR_SRC MOTOR_PWM_PCA9685
#else
#define MOTOR_SRC MOTOR_PWM_GPIO
#endif

static void init_motors(void)
{
    motor_init(&g_motors[MOTOR_FL], MOTOR_SRC,
               CFG_MOTOR_FL_A, CFG_MOTOR_FL_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_FL_INV);
    motor_init(&g_motors[MOTOR_FR], MOTOR_SRC,
               CFG_MOTOR_FR_A, CFG_MOTOR_FR_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_FR_INV);
    motor_init(&g_motors[MOTOR_RL], MOTOR_SRC,
               CFG_MOTOR_RL_A, CFG_MOTOR_RL_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_RL_INV);
    motor_init(&g_motors[MOTOR_RR], MOTOR_SRC,
               CFG_MOTOR_RR_A, CFG_MOTOR_RR_B,
               CFG_MOTOR_PWM_RANGE, CFG_MOTOR_RR_INV);
}

/* ================ 系统初始化 ================ */

static int sys_init(void)
{
    printf("===== Pacer v2.0 — 四轮差速驱动 =====\n");

    /* HAL */
    if (hal_gpio_init() < 0) {
        fprintf(stderr, "[APP] gpio init failed — are you root?\n");
        return -1;
    }

#if CFG_MOTOR_PWM_SRC == 1
    /* PCA9685 初始化 */
    if (hal_pca9685_init(CFG_PCA9685_I2C_BUS, CFG_PCA9685_I2C_ADDR,
                         CFG_MOTOR_PWM_HZ) != 0) {
        fprintf(stderr, "[APP] PCA9685 init failed\n");
        return -1;
    }
#endif

    /* 电机 */
    init_motors();

    /* 差速驱动 */
    diff_drive_init(&g_dd, g_motors);
    diff_drive_enable(&g_dd, true);

    printf("[APP] all systems ready\n");
    return 0;
}

static void sys_deinit(void)
{
    printf("[APP] cleaning up...\n");
    diff_drive_deinit(&g_dd);
#if CFG_MOTOR_PWM_SRC == 1
    hal_pca9685_deinit();
#endif
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
        usleep(50000);
    }

    imu_deinit();
    hal_gpio_deinit();
    return 0;
}

/* ================ 模式: 正常运行 ================ */

/**
 * 简单的键盘遥控 (临时方案)
 * TODO: 后续接入蓝牙手柄 / 手机APP / 自动追踪
 */
static void parse_keyboard_command(diff_drive_t *dd)
{
    fd_set fds;
    struct timeval tv = {0, 0};  /* 非阻塞 */
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
        char buf[8];
        if (fgets(buf, sizeof(buf), stdin)) {
            switch (buf[0]) {
                case 'w': case 'W': diff_drive_set(dd,  0.5f,  0.0f); break;
                case 's': case 'S': diff_drive_set(dd, -0.5f,  0.0f); break;
                case 'a': case 'A': diff_drive_set(dd,  0.0f, -0.5f); break;
                case 'd': case 'D': diff_drive_set(dd,  0.0f,  0.5f); break;
                case 'q': case 'Q': diff_drive_set(dd,  0.5f, -0.5f); break;
                case 'e': case 'E': diff_drive_set(dd,  0.5f,  0.5f); break;
                case ' ':           diff_drive_set(dd,  0.0f,  0.0f); break;
                case 'x': case 'X': g_running = 0; break;
            }
        }
    }
}

static int mode_run(void)
{
    if (sys_init() != 0) return -1;

    printf("\n[APP] 四轮差速驱动已启动!\n");
    printf("[APP] WASD 遥控, 空格=停, X=退出\n\n");

    /* 把 stdin 设为非阻塞 (raw 模式) */
    system("stty -echo -icanon min 0 time 0");

    /* === 主控制循环 === */
    int tick = 0;
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    long interval_us = 1000000L / CFG_DIFF_CONTROL_HZ;

    while (g_running) {
        /* 1. 读取遥控输入 */
        parse_keyboard_command(&g_dd);

        /* 2. 更新电机输出 */
        diff_drive_update(&g_dd, CFG_DIFF_CONTROL_DT);

        /* 3. 控制台日志 (2Hz) */
#if CFG_ENABLE_CONSOLE_LOG
        if (tick % (CFG_DIFF_CONTROL_HZ / 2) == 0) {
            printf("thr=%+5.2f steer=%+5.2f\n",
                   g_dd.throttle, g_dd.steering);
        }
#endif

        tick++;

        /* 4. 精确延时 */
        struct timespec t_next = t0;
        long add_ns = (long)tick * interval_us * 1000L;
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

    /* 恢复终端 */
    system("stty echo icanon");

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
