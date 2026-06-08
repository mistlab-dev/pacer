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

/* PWM 源: 0=GPIO直连, 1=PCA9685 */
#define CFG_MOTOR_PWM_SRC         1

/* PCA9685 */
#define CFG_PCA9685_I2C_BUS       1
#define CFG_PCA9685_I2C_ADDR      0x40

/* ==================== 电机驱动模式 ==================== */

/* 0=DC有刷 (双通道 PCA9685), 1=ESC无刷电调 (单通道) */
#define CFG_MOTOR_MODE             1

/* ESC 四轮通道 (每个电机只需 1 个 PCA9685 通道) */
#define CFG_ESC_FL_CH              0
#define CFG_ESC_FL_INV             false
#define CFG_ESC_FR_CH              1
#define CFG_ESC_FR_INV             true
#define CFG_ESC_RL_CH              2
#define CFG_ESC_RL_INV             false
#define CFG_ESC_RR_CH              3
#define CFG_ESC_RR_INV             true

/* DC 有刷通道 (备用, CFG_MOTOR_MODE=0 时使用) */
#define CFG_MOTOR_FL_A             0
#define CFG_MOTOR_FL_B             1
#define CFG_MOTOR_FL_INV           false
#define CFG_MOTOR_FR_A             2
#define CFG_MOTOR_FR_B             3
#define CFG_MOTOR_FR_INV           true
#define CFG_MOTOR_RL_A             4
#define CFG_MOTOR_RL_B             5
#define CFG_MOTOR_RL_INV           false
#define CFG_MOTOR_RR_A             6
#define CFG_MOTOR_RR_B             7
#define CFG_MOTOR_RR_INV           true

/* PCA9685 频率 — ESC 模式统一用 50Hz (舵机标准频率) */
#define CFG_PCA9685_PWM_HZ        50

/* ==================== 差速驱动 PID ==================== */

#define CFG_DIFF_SPEED_KP         1.0f
#define CFG_DIFF_SPEED_KI         0.0f
#define CFG_DIFF_SPEED_KD         0.0f
#define CFG_DIFF_CONTROL_HZ       100     /* 控制频率 */
#define CFG_DIFF_CONTROL_DT       (1.0f / CFG_DIFF_CONTROL_HZ)

/* ==================== 功能开关 ==================== */

#define CFG_USE_MADGWICK          1       /* 1=Madgwick, 0=互补滤波 */
#define CFG_ENABLE_CONSOLE_LOG    1       /* 控制台打印调试 */

/* ==================== 升降机构 (预留) ==================== */

#define CFG_LIFT_ENABLED          0       /* 0=暂不启用 */
#define CFG_LIFT_MOTOR_A          8       /* PCA9685 CH8 */
#define CFG_LIFT_MOTOR_B          9       /* PCA9685 CH9 */
#define CFG_LIFT_MAX_HEIGHT_CM    15.0f   /* 最大升高 (cm) */

/* ==================== 追踪 (四轮车暂不启用) ==================== */
#define CFG_TRACKER_ENABLED         0
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
