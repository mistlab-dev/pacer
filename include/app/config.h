/**
 * @file config.h
 * @brief 全局运行时配置
 *
 * 编译时常量放在这里。
 * 运行时参数通过 balance_set_xxx() 接口修改，方便在线调参。
 */

#ifndef PACER_CONFIG_H
#define PACER_CONFIG_H

/* ==================== IMU ==================== */

#define CFG_IMU_I2C_BUS           1       /* 树莓派 I2C-1 */
#define CFG_IMU_I2C_ADDR          0x68    /* ICM20948 AD0=0 */
#define CFG_IMU_SAMPLE_HZ         200     /* 采样率 */
#define CFG_IMU_ACCEL_RANGE_G     4       /* ±4g */
#define CFG_IMU_GYRO_RANGE_DPS    1000    /* ±1000°/s */

/* ==================== 控制频率 ==================== */

#define CFG_CONTROL_HZ            200     /* 主循环频率 (和 IMU 同步) */
#define CFG_CONTROL_DT            (1.0f / CFG_CONTROL_HZ)
#define CFG_CONTROL_INTERVAL_US   (1000000 / CFG_CONTROL_HZ)

/* ==================== 电机 ==================== */

#define CFG_MOTOR_PWM_HZ          20000   /* 20kHz 避免可闻噪声 */
#define CFG_MOTOR_PWM_RANGE       1000    /* 0~1000 分辨率 */
#define CFG_MOTOR_MAX_DUTY        900     /* 最大占空比 (留余量) */
#define CFG_MOTOR_DEADZONE        30      /* 死区补偿 */

/* 左电机: GPIO12, GPIO13 */
#define CFG_MOTOR_LEFT_A          12
#define CFG_MOTOR_LEFT_B          13
#define CFG_MOTOR_LEFT_INV        false

/* 右电机: GPIO18, GPIO19 (反向安装) */
#define CFG_MOTOR_RIGHT_A         18
#define CFG_MOTOR_RIGHT_B         19
#define CFG_MOTOR_RIGHT_INV       true

/* ==================== 平衡 PID 默认值 ==================== */

/* 角度环 (最关键) */
#define CFG_BAL_ANGLE_KP          25.0f
#define CFG_BAL_ANGLE_KI          0.0f
#define CFG_BAL_ANGLE_KD          0.8f
#define CFG_BAL_ANGLE_OFFSET      0.0f    /* 机械零点偏移 (度) */

/* 速度环 */
#define CFG_BAL_SPEED_KP          0.5f
#define CFG_BAL_SPEED_KI          0.01f
#define CFG_BAL_SPEED_KD          0.0f

/* 转向环 */
#define CFG_BAL_STEER_KP          0.3f
#define CFG_BAL_STEER_KD          0.05f

/* ==================== 安全限制 ==================== */

#define CFG_ANGLE_LIMIT_DEG       30.0f   /* 超出则限幅 */
#define CFG_ANGLE_EMERGENCY_DEG   45.0f   /* 超出则紧急停机 */

/* ==================== 功能开关 ==================== */

#define CFG_USE_MADGWICK          1       /* 1=Madgwick, 0=互补滤波 */
#define CFG_ENABLE_CONSOLE_LOG    1       /* 控制台打印调试 */

/* ==================== 追踪 ==================== */

#define CFG_TRACKER_ENABLED         1       /* 1=启用追踪, 0=纯平衡 */
#define CFG_TRACK_BEACON_UUID       "AA BB CC DD EE FF 00 11 22 33 44 55 66 77 88 99"
#define CFG_TRACK_HCI_DEVS          {0, 1, 2}  /* 3个蓝牙适配器 */
#define CFG_TRACK_FOLLOW_DIST       1.5f    /* 跟随距离 (米) */
#define CFG_TRACK_SPEED_MAX         0.5f    /* 最大跟随速度 (m/s) */
#define CFG_TRACK_TURN_MAX          1.0f    /* 最大转向速度 (rad/s) */
#define CFG_TRACK_LOST_TIMEOUT      3.0f    /* 丢失超时 (秒) */
#define CFG_TRACK_SCAN_SPEED        0.5f    /* 扫描旋转速度 */

/* 摄像头 */
#define CFG_CAMERA_DEVICE           0
#define CFG_CAMERA_WIDTH            320
#define CFG_CAMERA_HEIGHT           240
#define CFG_CAMERA_FPS              15
#define CFG_CAMERA_METHOD           0       /* 0=轮廓, 1=HOG */

#endif /* PACER_CONFIG_H */
