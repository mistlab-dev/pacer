/**
 * @file main.c
 * @brief Pacer - 二轮平衡车主程序
 *
 * 硬件: 树莓派 Zero 2W + ICM20948
 * 控制策略: 三环串级 PID (角度环 + 速度环 + 转向环)
 *
 * 使用方法:
 *   sudo ./pacer              # 正常启动
 *   sudo ./pacer --calibrate  # 校准 IMU
 *   sudo ./pacer --debug      # 调试模式 (打印数据)
 */

#include "config.h"
#include "sensors/imu.h"
#include "sensors/attitude.h"
#include "control/pid.h"
#include "control/balance.h"
#include "drivers/motor.h"
#include "utils/logger.h"

#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

/* ============== 全局状态 ============== */

static volatile int g_running = 1;
static balance_t g_balance;
static motor_t g_motor_left;
static motor_t g_motor_right;

/* ============== 信号处理 ============== */

static void signal_handler(int sig)
{
    printf("\n[MAIN] Signal %d received, shutting down...\n", sig);
    g_running = 0;
}

/* ============== 初始化 ============== */

static int init_all(void)
{
    printf("=== Pacer v1.0 — 二轮平衡车 ===\n");
    printf("[MAIN] Initializing...\n");

    /* pigpio 初始化 */
    if (gpioInitialise() < 0) {
        fprintf(stderr, "[MAIN] pigpio initialization failed!\n");
        fprintf(stderr, "       Make sure to run as root (sudo)\n");
        return -1;
    }
    printf("[MAIN] pigpio initialized\n");

    /* 日志 */
#if ENABLE_LOGGING
    logger_init(LOG_FILE_PATH);
#endif

    /* IMU */
    imu_config_t imu_cfg = IMU_DEFAULT_CONFIG;
    if (imu_init(&imu_cfg) < 0) {
        fprintf(stderr, "[MAIN] IMU init failed!\n");
        return -2;
    }

    /* 姿态解算 */
    attitude_init();

    /* 电机 */
    if (motor_init(&g_motor_left, MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B) < 0) {
        fprintf(stderr, "[MAIN] Left motor init failed!\n");
        return -3;
    }
    if (motor_init(&g_motor_right, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B) < 0) {
        fprintf(stderr, "[MAIN] Right motor init failed!\n");
        return -4;
    }

    /* 平衡控制器 */
    balance_init(&g_balance);

    printf("[MAIN] All systems initialized. Ready to balance!\n");
    return 0;
}

/* ============== 清理 ============== */

static void cleanup_all(void)
{
    printf("[MAIN] Cleaning up...\n");

    balance_stop(&g_balance);
    motor_close(&g_motor_left);
    motor_close(&g_motor_right);
    imu_close();
    logger_close();
    gpioTerminate();

    printf("[MAIN] Bye!\n");
}

/* ============== 校准模式 ============== */

static int do_calibrate(void)
{
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio init failed\n");
        return -1;
    }

    imu_config_t imu_cfg = IMU_DEFAULT_CONFIG;
    if (imu_init(&imu_cfg) < 0) {
        gpioTerminate();
        return -2;
    }

    printf("\n*** IMU 校准模式 ***\n");
    printf("请保持传感器静止，按 Enter 开始...\n");
    getchar();

    imu_calibrate(500);

    float offset[3];
    imu_get_gyro_offset(offset);
    printf("\n校准结果 (写入 config.h):\n");
    printf("  Gyro offset: X=%.3f, Y=%.3f, Z=%.3f deg/s\n",
           offset[0], offset[1], offset[2]);

    imu_close();
    gpioTerminate();
    return 0;
}

/* ============== 调试模式 ============== */

static int do_debug(void)
{
    if (init_all() < 0) return -1;

    printf("\n*** 调试模式 — 打印 IMU/姿态数据 ***\n");
    printf("按 Ctrl+C 退出\n\n");
    printf("%-10s %-10s %-10s %-10s %-10s %-10s\n",
           "Roll", "Pitch", "Yaw", "aX", "aY", "aZ");
    printf("------\n");

    imu_data_t imu;
    while (g_running) {
        if (imu_read(&imu) == 0) {
            attitude_t att = attitude_update_madgwick(&imu, 1.0f / ICM20948_SAMPLE_RATE);
            printf("%-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f\n",
                   att.roll, att.pitch, att.yaw,
                   imu.accel[0], imu.accel[1], imu.accel[2]);
        }
        usleep(50000); /* 50ms = 20Hz 显示 */
    }

    cleanup_all();
    return 0;
}

/* ============== 正常运行模式 ============== */

static int do_run(void)
{
    if (init_all() < 0) return -1;

    printf("\n*** 等待启动平衡 ***\n");
    printf("请将车体扶正，按 Enter 启动平衡控制...\n");
    getchar();

    /* 校准 */
    printf("[MAIN] Auto-calibrating gyro (keep still)...\n");
    imu_calibrate(200);
    usleep(100000);

    /* 启动平衡 */
    balance_start(&g_balance);

    printf("[MAIN] Balance active! Ctrl+C to stop.\n\n");

    imu_data_t imu;
    float dt = 1.0f / CONTROL_RATE_HZ;
    int loop_count = 0;

    struct timespec t_start, t_next, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while (g_running) {
        /* 计算下一帧时间 */
        t_next = t_start;
        t_next.tv_nsec += CONTROL_INTERVAL_US * 1000;
        if (t_next.tv_nsec >= 1000000000L) {
            t_next.tv_sec += t_next.tv_nsec / 1000000000L;
            t_next.tv_nsec %= 1000000000L;
        }

        /* 读取 IMU */
        if (imu_read(&imu) < 0) {
            LOG_W("IMU read failed");
            goto sleep_and_continue;
        }

        /* 姿态解算 */
        attitude_t att;
#if ENABLE_MADGWICK
        att = attitude_update_madgwick(&imu, dt);
#else
        att = attitude_update_complementary(&imu, dt);
#endif

        /* 平衡控制 */
        balance_update(&g_balance, &imu, &att, dt);

        /* 输出到电机 */
        if (!g_balance.emergency) {
            motor_set(&g_motor_left, g_balance.motor_left);
            motor_set(&g_motor_right, -g_balance.motor_right); /* 右侧镜像 */
        }

        /* 调试输出 (每 20 帧 = 10Hz @200Hz) */
        if (ENABLE_LOGGING && loop_count % 20 == 0) {
            printf("Roll=%7.2f° Target=%6.2f° L=%6.0f R=%6.0f\n",
                   g_balance.debug_angle,
                   g_balance.debug_angle_target,
                   g_balance.motor_left,
                   g_balance.motor_right);
        }

        loop_count++;

sleep_and_continue:
        /* 精确延时到下一帧 */
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        long wait_ns = (t_next.tv_sec - t_now.tv_sec) * 1000000000L
                      + (t_next.tv_nsec - t_now.tv_nsec);
        if (wait_ns > 0) {
            usleep(wait_ns / 1000);
        }
        t_start = t_next;
    }

    cleanup_all();
    return 0;
}

/* ============== 主入口 ============== */

static void print_usage(const char *prog)
{
    printf("Pacer v1.0 — 二轮平衡车 (RPi Zero 2W + ICM20948)\n\n");
    printf("Usage: sudo %s [options]\n\n", prog);
    printf("Options:\n");
    printf("  --calibrate   校准 IMU 陀螺仪\n");
    printf("  --debug       调试模式 (打印姿态数据，不驱动电机)\n");
    printf("  --help        显示帮助\n\n");
    printf("正常启动: sudo %s\n", prog);
}

int main(int argc, char *argv[])
{
    /* 参数解析 */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--calibrate") == 0) {
            return do_calibrate();
        } else if (strcmp(argv[i], "--debug") == 0) {
            signal(SIGINT, signal_handler);
            return do_debug();
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    /* 正常运行 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    return do_run();
}
