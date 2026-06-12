/**
 * @file config.h
 * @brief 全局运行时配置 — 四旋翼无人机
 *
 * 编译时常量。运行时参数通过接口函数修改，方便在线调参。
 */

#ifndef PACER_CONFIG_H
#define PACER_CONFIG_H

/* ==================== IMU ==================== */

#define CFG_IMU_I2C_BUS           1       /* 树莓派 I2C-1 */
#define CFG_IMU_I2C_ADDR          0x68    /* ICM20948 AD0=0 */
#define CFG_IMU_SAMPLE_HZ         400     /* 采样率 (四旋翼需要更高) */
#define CFG_IMU_ACCEL_RANGE_G     8       /* ±8g */
#define CFG_IMU_GYRO_RANGE_DPS    1000    /* ±1000°/s */

/* ==================== 控制频率 ==================== */

#define CFG_CONTROL_HZ            400     /* 主循环频率 */
#define CFG_CONTROL_DT            (1.0f / CFG_CONTROL_HZ)
#define CFG_CONTROL_INTERVAL_US   (1000000 / CFG_CONTROL_HZ)

/* ==================== 电机 ==================== */

/* 电机 PWM 频率 (DC 模式用, ESC 模式用 PCA9685 频率) */
#define CFG_MOTOR_PWM_HZ          20000
/* PCA9685 */
#define CFG_PCA9685_I2C_BUS       1
#define CFG_PCA9685_I2C_ADDR      0x40
#define CFG_PCA9685_PWM_HZ        400     /* 四旋翼 ESC 常用 400Hz (Oneshot125 可更高) */

/* ESC 四旋翼通道 */
#define CFG_ESC_FL_CH              0      /* 前左 — 顺时针 (CW) */
#define CFG_ESC_FR_CH              1      /* 前右 — 逆时针 (CCW) */
#define CFG_ESC_RL_CH              2      /* 后左 — 逆时针 (CCW) */
#define CFG_ESC_RR_CH              3      /* 后右 — 顺时针 (CW) */

/* PWM 参数 */
#define CFG_MOTOR_PWM_RANGE        1000

/* ==================== 姿态控制 PID ==================== */

/* 角速度环 (内环 — 核心) */
#define CFG_QUAD_ROLL_RATE_KP      0.40f
#define CFG_QUAD_ROLL_RATE_KI      0.05f
#define CFG_QUAD_ROLL_RATE_KD      0.01f

#define CFG_QUAD_PITCH_RATE_KP     0.40f
#define CFG_QUAD_PITCH_RATE_KI     0.05f
#define CFG_QUAD_PITCH_RATE_KD     0.01f

#define CFG_QUAD_YAW_RATE_KP      0.30f
#define CFG_QUAD_YAW_RATE_KI      0.02f
#define CFG_QUAD_YAW_RATE_KD      0.00f

/* 角度环 (外环 — 自稳模式) */
#define CFG_QUAD_ROLL_ANGLE_KP     4.0f
#define CFG_QUAD_ROLL_ANGLE_KI     0.0f
#define CFG_QUAD_ROLL_ANGLE_KD     0.0f

#define CFG_QUAD_PITCH_ANGLE_KP    4.0f
#define CFG_QUAD_PITCH_ANGLE_KI    0.0f
#define CFG_QUAD_PITCH_ANGLE_KD    0.0f

/* ==================== 运动限制 ==================== */

#define CFG_QUAD_MAX_ANGLE         30     /* 自稳模式最大倾斜角 (度) */
#define CFG_QUAD_MAX_ROLL_RATE     360    /* 最大 roll 角速度 (度/s) */
#define CFG_QUAD_MAX_PITCH_RATE    360    /* 最大 pitch 角速度 (度/s) */
#define CFG_QUAD_MAX_YAW_RATE      200    /* 最大 yaw 角速度 (度/s) */
#define CFG_QUAD_MAX_TILT_EMERGENCY 60    /* 超过此角度急停 (度) */

/* ==================== 混控 ==================== */

#define CFG_MIXER_THROTTLE_MIN     0.05f  /* 最低油门 (保持转动) */
#define CFG_MIXER_THROTTLE_MAX     1.0f
#define CFG_MIXER_HOVER            0.50f  /* 悬停油门估计 */

/* ==================== 遥控 ==================== */

/* 0=键盘 (调试用), 1=UDP (正常飞行) */
#define CFG_REMOTE_SRC             0
#define CFG_REMOTE_UDP_PORT        8888
#define CFG_REMOTE_TIMEOUT_SEC     0.5f

/* ==================== 功能开关 ==================== */

#define CFG_USE_MADGWICK           1      /* 1=Madgwick, 0=互补滤波 */
#define CFG_ENABLE_CONSOLE_LOG     1      /* 控制台打印调试 */

/* ==================== 安全 ==================== */

#define CFG_ARM_STICK_TIMEOUT_SEC  3.0f   /* 摇杆解锁窗口 */
#define CFG_NO_SIGNAL_TIMEOUT_SEC  0.5f   /* 无信号超时自动降落 */

#endif /* PACER_CONFIG_H */
