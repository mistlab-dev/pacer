/**
 * @file preflight.h
 * @brief 起飞前自检 — 硬件、传感器、安全检查
 *
 * 自检流程 (全部通过才允许起飞):
 *
 *   ┌──────────────────────────────────────────────────────┐
 *   │  1. 硬件连接   PCA9685 ↔ ESC ×4 通断检测            │
 *   │  2. IMU 健康   陀螺/加速度计量程合理性, 静止检测     │
 *   │  3. 校准状态   陀螺仪零偏是否已校准                  │
 *   │  4. 水平检测   倾斜角 < 阈值 (不能在斜坡起飞)        │
 *   │  5. 遥控连接   键盘/UDP 链路是否有效                 │
 *   │  6. 电机响应   ESC 已解锁, 最低油门确认              │
 *   │  7. 安全状态   无急停标志, 无倾斜保护触发            │
 *   └──────────────────────────────────────────────────────┘
 *
 * 每项返回 PASS / FAIL / WARN, 有详细诊断信息。
 */

#ifndef PACER_PREFLIGHT_H
#define PACER_PREFLIGHT_H

#include <stdbool.h>
#include "sensor/imu.h"
#include "filter/filter.h"

/* 检查项 */
typedef enum {
    CHECK_HW_PCA9685,    /* PCA9685 连接 */
    CHECK_HW_ESC,        /* ESC 解锁状态 */
    CHECK_IMU_CONNECT,   /* IMU 通信 */
    CHECK_IMU_RANGE,     /* IMU 数据合理性 */
    CHECK_IMU_STATIC,    /* 静止检测 */
    CHECK_GYRO_CAL,      /* 陀螺仪校准 */
    CHECK_LEVEL,         /* 水平检测 */
    CHECK_REMOTE,        /* 遥控连接 */
    CHECK_SAFETY,        /* 安全状态 */
    CHECK_COUNT          /* 总数 */
} preflight_check_t;

/* 检查结果 */
typedef enum {
    CHECK_PASS,   /* 通过 */
    CHECK_FAIL,   /* 失败 — 禁止起飞 */
    CHECK_WARN,   /* 警告 — 可以飞但要注意 */
} check_result_t;

/* 单项检查结果 */
typedef struct {
    check_result_t result;
    char message[128];   /* 诊断信息 */
} check_status_t;

/* 整体自检报告 */
typedef struct {
    check_status_t items[CHECK_COUNT];
    bool all_passed;      /* 全部 PASS (允许 WARN) */
    bool has_failure;     /* 至少一项 FAIL */
    int  pass_count;
    int  warn_count;
    int  fail_count;
} preflight_report_t;

/**
 * @brief 执行完整自检
 *
 * 依次执行所有检查项, 填充报告。
 * 调用者根据 report.all_passed 决定是否允许起飞。
 *
 * @param report     输出: 自检报告
 * @param imu_ok     IMU 读取是否成功 (0=OK)
 * @param sample     最新 IMU 样本
 * @param att        当前姿态估计
 * @param gyro_calibrated  陀螺仪是否已校准
 * @param gyro_bias        当前零偏
 * @param remote_connected 遥控是否连接
 * @param esc_armed        ESC 是否已解锁
 * @param pca9685_ok       PCA9685 是否正常
 */
void preflight_run(preflight_report_t *report,
                   int imu_ok,
                   const imu_sample_t *sample,
                   const attitude_t *att,
                   bool gyro_calibrated,
                   const vec3_t *gyro_bias,
                   bool remote_connected,
                   bool esc_armed,
                   bool pca9685_ok);

/**
 * @brief 打印自检报告
 */
void preflight_print(const preflight_report_t *report);

/**
 * @brief 检查项名称
 */
const char *preflight_check_name(preflight_check_t check);

/**
 * @brief 结果文本
 */
const char *check_result_str(check_result_t r);

#endif /* PACER_PREFLIGHT_H */
