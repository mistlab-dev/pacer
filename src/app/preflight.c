/**
 * @file preflight.c
 * @brief 起飞前自检实现
 *
 * 每项检查独立判定, 互不依赖 (除了最终汇总)。
 * WARN 不阻断起飞, FAIL 阻断。
 */

#include "app/preflight.h"
#include "app/config.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ============ 阈值 ============ */

#define LEVEL_MAX_TILT_DEG     10.0f    /* 水平检测: 最大允许倾斜 */
#define STATIC_GYRO_MAX_DPS    3.0f     /* 静止检测: 陀螺仪最大读数 */
#define STATIC_ACCEL_MIN_G     0.85f    /* 加速度下限 */
#define STATIC_ACCEL_MAX_G     1.15f    /* 加速度上限 */
#define GYRO_BIAS_MAX_DPS      2.0f     /* 校准: 零偏超此值算未校准 */
#define ACCEL_NOISE_THRESHOLD  15.0f    /* 加速度量程上限 m/s² (检测异常) */

/* ============ 内部工具 ============ */

static float vec3f_abs(const vec3_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static float vec3f_max_abs(const vec3_t *v)
{
    float ax = fabsf(v->x);
    float ay = fabsf(v->y);
    float az = fabsf(v->z);
    float m = ax > ay ? ax : ay;
    return m > az ? m : az;
}

static float accel_magnitude_g(const vec3_t *a)
{
    /* IMU 加速度单位是 m/s², 1g ≈ 9.81 m/s² */
    return vec3f_abs(a) / 9.81f;
}

/* ============ 检查项 ============ */

static void check_hardware(preflight_report_t *r, bool pca9685_ok, bool esc_armed)
{
    /* PCA9685 */
    if (pca9685_ok) {
        r->items[CHECK_HW_PCA9685] = (check_status_t){
            CHECK_PASS, "PCA9685 I2C 通信正常"
        };
    } else {
        r->items[CHECK_HW_PCA9685] = (check_status_t){
            CHECK_FAIL, "PCA9685 通信失败! 检查 I2C 接线和地址"
        };
    }

    /* ESC */
    if (esc_armed) {
        r->items[CHECK_HW_ESC] = (check_status_t){
            CHECK_PASS, "ESC 已解锁 (最低油门信号)"
        };
    } else {
        r->items[CHECK_HW_ESC] = (check_status_t){
            CHECK_FAIL, "ESC 未解锁! 请先执行 ESC arm 流程"
        };
    }
}

static void check_imu(preflight_report_t *r, int imu_ok,
                      const imu_sample_t *sample)
{
    /* 通信 */
    if (imu_ok != 0) {
        r->items[CHECK_IMU_CONNECT] = (check_status_t){
            CHECK_FAIL, "IMU 读取失败! 检查 ICM-20948 连接"
        };
        r->items[CHECK_IMU_RANGE] = (check_status_t){
            CHECK_FAIL, "IMU 无数据, 无法检查"
        };
        r->items[CHECK_IMU_STATIC] = (check_status_t){
            CHECK_FAIL, "IMU 无数据, 无法检查"
        };
        return;
    }
    r->items[CHECK_IMU_CONNECT] = (check_status_t){
        CHECK_PASS, "IMU 通信正常"
    };

    /* 数据合理性: 加速度量级在 0.5~1.5g, 陀螺不超量程 */
    float accel_g = accel_magnitude_g(&sample->accel);
    float gyro_max = vec3f_max_abs(&sample->gyro);

    if (accel_g < 0.3f || accel_g > 2.0f) {
        char msg[128];
        snprintf(msg, sizeof(msg), "加速度异常: %.2fg (正常应 ~1.0g)", accel_g);
        r->items[CHECK_IMU_RANGE] = (check_status_t){CHECK_FAIL, {0}};
        strncpy(r->items[CHECK_IMU_RANGE].message, msg, 127);
    } else if (gyro_max > (float)CFG_IMU_GYRO_RANGE_DPS) {
        char msg[128];
        snprintf(msg, sizeof(msg), "陀螺仪超量程: %.0f°/s (量程 ±%d°/s)",
                 gyro_max, CFG_IMU_GYRO_RANGE_DPS);
        r->items[CHECK_IMU_RANGE] = (check_status_t){CHECK_FAIL, {0}};
        strncpy(r->items[CHECK_IMU_RANGE].message, msg, 127);
    } else {
        r->items[CHECK_IMU_RANGE] = (check_status_t){
            CHECK_PASS, "IMU 数据在合理范围"
        };
    }

    /* 静止检测 */
    bool gyro_quiet = gyro_max < STATIC_GYRO_MAX_DPS;
    bool accel_normal = (accel_g > STATIC_ACCEL_MIN_G && accel_g < STATIC_ACCEL_MAX_G);

    if (gyro_quiet && accel_normal) {
        r->items[CHECK_IMU_STATIC] = (check_status_t){
            CHECK_PASS, "传感器静止, 加速度正常"
        };
    } else if (!gyro_quiet) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "陀螺仪偏大: %.1f°/s (阈值 %.0f°/s) — 请保持静止",
                 gyro_max, (float)STATIC_GYRO_MAX_DPS);
        r->items[CHECK_IMU_STATIC] = (check_status_t){CHECK_WARN, {0}};
        strncpy(r->items[CHECK_IMU_STATIC].message, msg, 127);
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "加速度异常: %.2fg — 检查安装方向或震动",
                 accel_g);
        r->items[CHECK_IMU_STATIC] = (check_status_t){CHECK_WARN, {0}};
        strncpy(r->items[CHECK_IMU_STATIC].message, msg, 127);
    }
}

static void check_calibration(preflight_report_t *r,
                               bool gyro_calibrated,
                               const vec3_t *gyro_bias)
{
    if (!gyro_calibrated) {
        r->items[CHECK_GYRO_CAL] = (check_status_t){
            CHECK_WARN, "陀螺仪未校准 — 建议运行 --calibrate"
        };
        return;
    }

    float bias_max = vec3f_max_abs(gyro_bias);
    if (bias_max > GYRO_BIAS_MAX_DPS) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "零偏偏大: %.2f°/s — 建议重新校准", bias_max);
        r->items[CHECK_GYRO_CAL] = (check_status_t){CHECK_WARN, {0}};
        strncpy(r->items[CHECK_GYRO_CAL].message, msg, 127);
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "陀螺仪已校准, 最大零偏 %.2f°/s", bias_max);
        r->items[CHECK_GYRO_CAL] = (check_status_t){CHECK_PASS, {0}};
        strncpy(r->items[CHECK_GYRO_CAL].message, msg, 127);
    }
}

static void check_level(preflight_report_t *r, const attitude_t *att)
{
    float tilt = sqrtf(att->roll * att->roll + att->pitch * att->pitch);

    if (tilt < LEVEL_MAX_TILT_DEG) {
        r->items[CHECK_LEVEL] = (check_status_t){
            CHECK_PASS, "水平状态良好"
        };
    } else if (tilt < 20.0f) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "略有倾斜: %.1f° (阈值 %.0f°) — 注意起飞方向",
                 tilt, LEVEL_MAX_TILT_DEG);
        r->items[CHECK_LEVEL] = (check_status_t){CHECK_WARN, {0}};
        strncpy(r->items[CHECK_LEVEL].message, msg, 127);
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "倾斜过大: %.1f° — 请在水平面起飞!", tilt);
        r->items[CHECK_LEVEL] = (check_status_t){CHECK_FAIL, {0}};
        strncpy(r->items[CHECK_LEVEL].message, msg, 127);
    }
}

static void check_remote(preflight_report_t *r, bool remote_connected)
{
    if (remote_connected) {
        r->items[CHECK_REMOTE] = (check_status_t){
            CHECK_PASS, "遥控已连接"
        };
    } else {
        r->items[CHECK_REMOTE] = (check_status_t){
            CHECK_WARN, "遥控未连接 — 起飞前请先连接"
        };
    }
}

static void check_safety(preflight_report_t *r, bool tilt_emergency)
{
    if (!tilt_emergency) {
        r->items[CHECK_SAFETY] = (check_status_t){
            CHECK_PASS, "安全状态正常"
        };
    } else {
        r->items[CHECK_SAFETY] = (check_status_t){
            CHECK_FAIL, "安全保护已触发! 请重置后起飞"
        };
    }
}

/* ============ 汇总 ============ */

static void summarize(preflight_report_t *r)
{
    r->pass_count = 0;
    r->warn_count = 0;
    r->fail_count = 0;

    for (int i = 0; i < CHECK_COUNT; i++) {
        switch (r->items[i].result) {
            case CHECK_PASS: r->pass_count++; break;
            case CHECK_WARN: r->warn_count++; break;
            case CHECK_FAIL: r->fail_count++; break;
        }
    }

    r->all_passed  = (r->fail_count == 0);
    r->has_failure = (r->fail_count > 0);
}

/* ============ 公开接口 ============ */

void preflight_run(preflight_report_t *report,
                   int imu_ok,
                   const imu_sample_t *sample,
                   const attitude_t *att,
                   bool gyro_calibrated,
                   const vec3_t *gyro_bias,
                   bool remote_connected,
                   bool esc_armed,
                   bool pca9685_ok)
{
    memset(report, 0, sizeof(*report));

    check_hardware(report, pca9685_ok, esc_armed);
    check_imu(report, imu_ok, sample);
    check_calibration(report, gyro_calibrated, gyro_bias);
    check_level(report, att);
    check_remote(report, remote_connected);
    check_safety(report, false);  /* tilt_emergency 由上层传入, 初始 false */

    summarize(report);
}

void preflight_print(const preflight_report_t *r)
{
    printf("\n╔══════════════════════════════════════════════╗\n");
    printf("║         起飞前自检报告 (Preflight)           ║\n");
    printf("╠══════════════════════════════════════════════╣\n");

    static const char *icons[] = {"✅", "⚠️ ", "❌"};

    for (int i = 0; i < CHECK_COUNT; i++) {
        printf("║ %s %-20s %s\n",
               icons[r->items[i].result],
               preflight_check_name((preflight_check_t)i),
               r->items[i].message);
    }

    printf("╠══════════════════════════════════════════════╣\n");
    printf("║  通过 %d · 警告 %d · 失败 %d / 共 %d 项\n",
           r->pass_count, r->warn_count, r->fail_count, CHECK_COUNT);
    printf("║  %s\n", r->all_passed ? "🟢 可以起飞" : "🔴 禁止起飞 — 请修复 FAIL 项");
    printf("╚══════════════════════════════════════════════╝\n\n");
}

const char *preflight_check_name(preflight_check_t check)
{
    static const char *names[] = {
        [CHECK_HW_PCA9685]  = "PCA9685",
        [CHECK_HW_ESC]      = "ESC 解锁",
        [CHECK_IMU_CONNECT] = "IMU 通信",
        [CHECK_IMU_RANGE]   = "IMU 量程",
        [CHECK_IMU_STATIC]  = "静止检测",
        [CHECK_GYRO_CAL]    = "陀螺校准",
        [CHECK_LEVEL]       = "水平状态",
        [CHECK_REMOTE]      = "遥控连接",
        [CHECK_SAFETY]      = "安全状态",
    };
    if (check < CHECK_COUNT) return names[check];
    return "未知";
}

const char *check_result_str(check_result_t r)
{
    switch (r) {
        case CHECK_PASS: return "PASS";
        case CHECK_WARN: return "WARN";
        case CHECK_FAIL: return "FAIL";
    }
    return "?";
}
