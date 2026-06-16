/**
 * @file imu_debug.h
 * @brief IMU 调试工具集 — 校准、诊断、数据记录/回放、频谱分析
 *
 * 独立模块，不参与飞控主循环。
 * 通过命令行参数触发 (--debug-imu, --calibrate, --imu-spectrum 等)。
 *
 * 功能清单:
 *
 *   1. 实时数据流   — 滚动打印 6 轴数据 + 姿态角
 *   2. 校准向导     — 陀螺零偏 + 加速度计零偏 + 磁力计校准
 *   3. 自检诊断     — WHO_AM_I + 通信完整性 + 数据合理性
 *   4. 频谱分析     — FFT 看振动频率分布 (找共振点)
 *   5. 数据记录     — CSV 导出, 方便离线分析
 *   6. 回放         — 读 CSV 重放, 对比滤波器效果
 *   7. 滤波器对比   — Madgwick vs 互补滤波 并排显示
 *   8. 静态噪声     — 统计陀螺/加速度噪声 RMS
 */

#ifndef PACER_IMU_DEBUG_H
#define PACER_IMU_DEBUG_H

#include "sensor/imu.h"

/* ---------- 调试模式 ---------- */

typedef enum {
    IMU_DBG_STREAM,       /* 实时数据流 */
    IMU_DBG_CALIBRATE,    /* 校准向导 */
    IMU_DBG_DIAGNOSE,     /* 自检诊断 */
    IMU_DBG_SPECTRUM,     /* 频谱分析 */
    IMU_DBG_RECORD,       /* CSV 记录 */
    IMU_DBG_REPLAY,       /* CSV 回放 */
    IMU_DBG_COMPARE,      /* 滤波器对比 */
    IMU_DBG_NOISE,        /* 静态噪声分析 */
} imu_debug_mode_t;

/**
 * @brief IMU 调试入口
 * @param mode  调试模式
 * @param arg   可选参数 (如 CSV 文件路径)
 * @return 退出码
 */
int imu_debug_run(imu_debug_mode_t mode, const char *arg);

#endif /* PACER_IMU_DEBUG_H */
