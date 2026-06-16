/**
 * @file app.c
 * @brief 应用层 — 四旋翼无人机 (RPi Zero 2W + ICM20948 + PCA9685)
 *
 * 完整飞行流程:
 *
 *   ┌──────┐    ┌─────────┐    ┌────────┐    ┌─────────┐    ┌──────────┐
 *   │ IDLE │───→│ PREFLIGHT│───→│ TAKEOFF│───→│ HOVER   │───→│ LANDING  │
 *   │ 待命  │    │ 自检     │    │ 起飞   │    │ 悬停    │    │ 降落     │
 *   └──────┘    └─────────┘    └────────┘    └────────┘    └──────────┘
 *                                                       ↓
 *                                                    GROUNDED
 *
 * 主循环:
 *   1. 读取 IMU
 *   2. 姿态估计
 *   3. 安全检查 (倾斜保护)
 *   4. 遥控输入
 *   5. 飞行阶段状态机 → 油门目标
 *   6. 姿态控制器 (串级 PID)
 *   7. 混控 → 电机输出
 */

#include "app/app.h"
#include "app/config.h"
#include "app/preflight.h"
#include "app/flight_phase.h"

#include "hal/hal_gpio.h"
#include "hal/hal_i2c.h"
#include "hal/hal_pca9685.h"
#include "sensor/imu.h"
#include "sensor/imu_icm20948.h"
#include "filter/filter.h"
#include "ctrl/pid.h"
#include "ctrl/attitude.h"
#include "ctrl/quad_mixer.h"
#include "motor/motor.h"
#include "remote/remote.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>

/* ================ 全局状态 ================ */

static volatile int g_running = 1;

static attitude_ctrl_t   g_att;
static quad_mixer_t      g_mixer;
static motor_t           g_motors[QUAD_NUM_MOTORS];
static remote_config_t   g_remote_cfg;
static flight_phase_t    g_flight;

/* IMU 校准状态 */
static bool   g_gyro_calibrated = false;
static vec3_t g_gyro_bias       = {0};

/* ================ 信号 ================ */

static void on_signal(int sig)
{
    printf("\n[APP] signal %d, shutting down...\n", sig);
    g_running = 0;
}

/* ================ 初始化电机 ================ */

static void init_motors(void)
{
    motor_init_esc(&g_motors[QUAD_FL], CFG_ESC_FL_CH, false);
    motor_init_esc(&g_motors[QUAD_FR], CFG_ESC_FR_CH, false);
    motor_init_esc(&g_motors[QUAD_RL], CFG_ESC_RL_CH, false);
    motor_init_esc(&g_motors[QUAD_RR], CFG_ESC_RR_CH, false);
}

/* ================ ESC 解锁 ================ */

static void esc_arm_all(void)
{
    printf("[APP] ESC 解锁中...\n");
    for (int i = 0; i < QUAD_NUM_MOTORS; i++) {
        motor_esc_arm(&g_motors[i]);
    }
    printf("[APP] 等待 3 秒...\n");
    usleep(3000000);
    printf("[APP] ESC 解锁完成!\n");
}

/* ================ 系统初始化 ================ */

static int sys_init(void)
{
    printf("===== Pacer v3.0 — 四旋翼无人机 =====\n");

    /* HAL */
    if (hal_gpio_init() < 0) {
        fprintf(stderr, "[APP] gpio init failed — are you root?\n");
        return -1;
    }

    /* PCA9685 */
    if (hal_pca9685_init(CFG_PCA9685_I2C_BUS, CFG_PCA9685_I2C_ADDR,
                         CFG_PCA9685_PWM_HZ) != 0) {
        fprintf(stderr, "[APP] PCA9685 init failed\n");
        return -1;
    }

    /* 电机 */
    init_motors();
    esc_arm_all();

    /* 姿态控制器 */
    attitude_init(&g_att);

    /* 混控器 */
    mixer_config_t mcfg = MIXER_CONFIG_DEFAULT;
    quad_mixer_init(&g_mixer, &mcfg);

    /* 飞行阶段状态机 */
    flight_phase_config_t fpcfg = FLIGHT_PHASE_CONFIG_DEFAULT;
    fpcfg.takeoff_throttle    = CFG_TAKEOFF_THROTTLE;
    fpcfg.hover_throttle      = CFG_HOVER_THROTTLE;
    fpcfg.takeoff_ramp_rate   = CFG_TAKEOFF_RAMP_RATE;
    fpcfg.landing_descent_rate = CFG_LANDING_DESCENT_RATE;
    fpcfg.landing_throttle_min = CFG_LANDING_THROTTLE_MIN;
    fpcfg.hover_max_drift_deg = CFG_HOVER_MAX_DRIFT_DEG;
    fpcfg.tilt_emergency_deg  = (float)CFG_QUAD_MAX_TILT_EMERGENCY;
    flight_phase_init(&g_flight, &fpcfg);

    printf("[APP] all systems ready\n");
    return 0;
}

static void sys_deinit(void)
{
    printf("[APP] cleaning up...\n");

    for (int i = 0; i < QUAD_NUM_MOTORS; i++) {
        motor_set(&g_motors[i], 0.0f);
    }

    quad_mixer_deinit(&g_mixer);
    attitude_enable(&g_att, false);
    hal_pca9685_deinit();
    hal_gpio_deinit();
    printf("[APP] bye\n");
}

/* ================ 模式: 校准 IMU ================ */

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

/* ================ 模式: ESC 校准 ================ */

static int mode_esc_cal(void)
{
    printf("\n=== ESC 油门行程校准 ===\n");
    printf("这个流程会: 最大油门 → 接通 ESC 电源 → 听到滴滴 → 拉到最低\n\n");

    if (hal_gpio_init() < 0) return -1;

    if (hal_pca9685_init(CFG_PCA9685_I2C_BUS, CFG_PCA9685_I2C_ADDR,
                         CFG_PCA9685_PWM_HZ) != 0) {
        hal_gpio_deinit();
        return -1;
    }

    motor_t m[4];
    for (int i = 0; i < 4; i++) {
        motor_init_esc(&m[i], i, false);
    }

    printf("步骤 1: 发送最大油门到所有 ESC\n");
    printf("请断开 ESC 电源, 按 Enter 继续...\n");
    getchar();

    for (int i = 0; i < 4; i++) {
        motor_set(&m[i], 1.0f);
    }

    printf("现在接通 ESC 电源! 等待滴滴声...\n");
    printf("听到两声短滴后, 按 Enter 发送最小油门...\n");
    getchar();

    for (int i = 0; i < 4; i++) {
        motor_set(&m[i], 0.0f);
    }

    printf("等待 3 秒确认...\n");
    usleep(3000000);

    printf("校准完成! 断开电源再重新接通即可使用。\n");

    for (int i = 0; i < 4; i++) {
        motor_deinit(&m[i]);
    }
    hal_pca9685_deinit();
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
           "Roll", "Pitch", "Yaw", "gX", "gY", "gZ");
    printf("─────────────────────────────────────────────\n");

    imu_sample_t sample;
    while (g_running) {
        if (imu_read(&sample) == 0) {
            attitude_t att = filter_update(&sample, CFG_CONTROL_DT);
            printf("%-8.2f %-8.2f %-8.2f  | %-8.1f %-8.1f %-8.1f\n",
                   att.roll, att.pitch, att.yaw,
                   sample.gyro.x, sample.gyro.y, sample.gyro.z);
        }
        usleep(50000);
    }

    imu_deinit();
    hal_gpio_deinit();
    return 0;
}

/* ================ 模式: 正常飞行 ================ */

static int mode_run(void)
{
    if (sys_init() != 0) return -1;

    /* IMU */
    imu_icm20948_register();
    imu_config_t imu_cfg = IMU_CONFIG_DEFAULT;
    if (imu_init(&imu_cfg) != 0) {
        fprintf(stderr, "[APP] IMU init failed\n");
        sys_deinit();
        return -2;
    }

    /* 姿态滤波器 */
    filter_config_t f_cfg = FILTER_CONFIG_DEFAULT;
    filter_init(&f_cfg);

    /* 遥控 */
    g_remote_cfg = (remote_config_t)REMOTE_CONFIG_DEFAULT;
#if CFG_REMOTE_SRC == 1
    g_remote_cfg.source = REMOTE_SRC_UDP;
#endif
    remote_init(&g_remote_cfg);

    /* ---- 第一步: 自检 ---- */
    printf("\n[APP] 按 Enter 执行起飞前自检...\n");
    getchar();

    imu_sample_t sample;
    int imu_ok = imu_read(&sample);
    attitude_t att_zero = {0};

    preflight_report_t report;
    preflight_run(&report,
                  imu_ok, &sample, &att_zero,
                  g_gyro_calibrated, &g_gyro_bias,
                  true,   /* remote: 键盘模式始终 true */
                  true,   /* esc_armed */
                  true);  /* pca9685_ok */

    preflight_print(&report);

    if (!report.all_passed) {
        fprintf(stderr, "[APP] 自检未通过, 起飞取消!\n");
        remote_deinit();
        imu_deinit();
        sys_deinit();
        return 1;
    }

    printf("[APP] 自检通过! 按 Enter 起飞...\n");
    getchar();

    /* ---- 第二步: 解锁 + 起飞 ---- */
    quad_mixer_arm(&g_mixer);
    attitude_enable(&g_att, true);
    attitude_set_mode(&g_att, ATT_MODE_ANGLE);  /* 自稳模式 */
    flight_phase_request_takeoff(&g_flight);

    printf("\n[APP] 四旋翼飞控已启动!\n");
    printf("[APP] WASD=姿态  QE=偏航  1-9=油门叠加  0/空格=悬停  X=急停  L=降落\n\n");

    /* === 主控制循环 === */
    int tick = 0;
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    long interval_us = 1000000L / CFG_CONTROL_HZ;

    bool armed = true;
    bool was_armed = true;
    float manual_throttle_boost = 0.0f;  /* 手动油门叠加 */

    while (g_running) {
        /* 1. 读取 IMU */
        int imu_ok = imu_read(&sample);

        /* 2. 姿态估计 */
        attitude_t att = {0};
        if (imu_ok == 0) {
            att = filter_update(&sample, CFG_CONTROL_DT);
        }

        /* 3. 遥控输入 */
        remote_cmd_t cmd;
        remote_poll(&cmd);

        if (cmd.estop) {
            printf("[APP] *** E-STOP ***\n");
            flight_phase_emergency(&g_flight);
            armed = false;
            quad_mixer_disarm(&g_mixer);
            attitude_enable(&g_att, false);
            break;
        }

        /* 降落指令 */
        if (cmd.throttle < 0.01f && flight_phase_in_flight(&g_flight)) {
            /* 油门拉到零 → 降落 */
            if (g_flight.phase == PHASE_HOVER || g_flight.phase == PHASE_TAKEOFF) {
                flight_phase_request_landing(&g_flight);
            }
        }

        /* 油门叠加: 1-9 键在悬停基础上微调 */
        manual_throttle_boost = cmd.throttle - 0.5f;  /* 遥控油门中位=0 */

        /* 解锁/上锁切换 */
        if (cmd.arm_switch && !was_armed) {
            if (g_flight.phase == PHASE_GROUNDED) {
                flight_phase_reset(&g_flight);
                quad_mixer_arm(&g_mixer);
                attitude_enable(&g_att, true);
                armed = true;
                printf("[APP] *** RE-ARMED ***\n");
            }
        } else if (!cmd.arm_switch && was_armed) {
            if (flight_phase_in_flight(&g_flight)) {
                flight_phase_request_landing(&g_flight);
            } else {
                quad_mixer_disarm(&g_mixer);
                attitude_enable(&g_att, false);
                armed = false;
            }
        }
        was_armed = cmd.arm_switch;

        /* 4. 飞行阶段状态机 → 油门目标 */
        float phase_throttle = flight_phase_update(&g_flight,
                                                    att.roll, att.pitch,
                                                    CFG_CONTROL_DT);

        /* 已着陆 → 上锁 */
        if (g_flight.phase == PHASE_GROUNDED && armed) {
            quad_mixer_disarm(&g_mixer);
            attitude_enable(&g_att, false);
            armed = false;
            printf("[APP] *** 已着陆, 电机已停 ***\n");
            printf("[APP] 按 R 解锁重新飞行, Ctrl+C 退出\n");
        }

        /* 5. 姿态控制 */
        if (imu_ok == 0 && armed && g_flight.phase != PHASE_IDLE) {
            /* 悬停时: 摇杆输入作为微调, 油门来自状态机 */
            float ctrl_throttle = phase_throttle + manual_throttle_boost;
            ctrl_throttle = fmaxf(0.0f, fminf(1.0f, ctrl_throttle));

            attitude_set_input(&g_att,
                               ctrl_throttle,
                               cmd.roll,
                               cmd.pitch,
                               cmd.yaw);

            const attitude_output_t *att_out =
                attitude_update(&g_att, &att, &sample, CFG_CONTROL_DT);

            /* 6. 混控 */
            mixer_output_t mix_out = quad_mixer_update(&g_mixer,
                att_out->throttle,
                att_out->roll,
                att_out->pitch,
                att_out->yaw);

            /* 7. 驱动电机 */
            for (int i = 0; i < QUAD_NUM_MOTORS; i++) {
                motor_set(&g_motors[i], mix_out.motor[i]);
            }

#if CFG_ENABLE_CONSOLE_LOG
            if (tick % (CFG_CONTROL_HZ / 2) == 0) {  /* 2Hz 打印 */
                printf("[%s] R:%+6.1f P:%+6.1f | thr:%.2f [%.2f %.2f %.2f %.2f] %.1fs\n",
                       flight_phase_name(g_flight.phase),
                       att.roll, att.pitch,
                       att_out->throttle,
                       mix_out.motor[0], mix_out.motor[1],
                       mix_out.motor[2], mix_out.motor[3],
                       g_flight.phase_time);
            }
#endif
        } else {
            /* 未飞行: 电机零输出 */
            for (int i = 0; i < QUAD_NUM_MOTORS; i++) {
                motor_set(&g_motors[i], 0.0f);
            }
        }

        tick++;

        /* 定时睡眠 */
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

    /* 清理 */
    remote_deinit();
    imu_deinit();
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
        case APP_MODE_ESC_CAL:   return mode_esc_cal();
        case APP_MODE_RUN:       return mode_run();
    }
    return 1;
}
