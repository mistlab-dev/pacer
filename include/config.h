/**
 * @file config.h
 * @brief Pacer - 二轮平衡车配置参数
 */

#ifndef PACER_CONFIG_H
#define PACER_CONFIG_H

/* ======================== 硬件引脚配置 ======================== */

/* ICM20948 IMU - I2C */
#define ICM20948_I2C_BUS        1       // I2C-1 (GPIO2=SDA, GPIO3=SCL)
#define ICM20948_I2C_ADDR       0x68    // AD0=0; AD0=1 -> 0x69
#define ICM20948_SAMPLE_RATE    200     // Hz (陀螺仪/加速度计采样率)

/* 电机 PWM - 左 */
#define MOTOR_LEFT_PIN_A        12      // GPIO12 (PWM0)
#define MOTOR_LEFT_PIN_B        13      // GPIO13 (PWM1)

/* 电机 PWM - 右 */
#define MOTOR_RIGHT_PIN_A       18      // GPIO18 (PWM0)
#define MOTOR_RIGHT_PIN_B       19      // GPIO19 (PWM1)

/* 编码器 - 左 (可选) */
#define ENCODER_LEFT_PIN_A      5       // GPIO5
#define ENCODER_LEFT_PIN_B      6       // GPIO6

/* 编码器 - 右 (可选) */
#define ENCODER_RIGHT_PIN_A     16      // GPIO16
#define ENCODER_RIGHT_PIN_B     26      // GPIO26

/* 电池电压监测 */
#define BATTERY_ADC_PIN         -1      // 暂未使用

/* ======================== 电机参数 ======================== */

#define MOTOR_PWM_FREQ          20000   // PWM 频率 20kHz
#define MOTOR_PWM_RANGE         1000    // PWM 范围 0-1000
#define MOTOR_MAX_DUTY          900     // 最大占空比 (防止过载)
#define MOTOR_MIN_DUTY          30      // 最小启动占空比

/* ======================== PID 参数 ======================== */

/* 平衡环 (角度 PID) — 最核心 */
#define BALANCE_KP              25.0f
#define BALANCE_KI              0.0f
#define BALANCE_KD              0.8f
#define BALANCE_ANGLE_OFFSET    0.0f    // 机械零点偏移 (度)

/* 速度环 */
#define SPEED_KP                0.5f
#define SPEED_KI                0.01f
#define SPEED_KD                0.0f
#define SPEED_MAX_OUTPUT        10.0f   // 速度环最大输出 (度)

/* 转向环 */
#define STEER_KP                0.3f
#define STEER_KD                0.05f

/* ======================== 控制参数 ======================== */

#define CONTROL_RATE_HZ         200     // 主控制循环频率 (Hz)
#define CONTROL_INTERVAL_US     (1000000 / CONTROL_RATE_HZ)

#define ANGLE_MAX               30.0f   // 最大允许倾斜角度 (度)
#define ANGLE_EMERGENCY         45.0f   // 紧急停机角度 (度)

/* 互补滤波系数 (姿态解算) */
#define COMPLEMENTARY_ALPHA     0.98f

/* Madgwick 滤波器参数 */
#define MADGWICK_BETA           0.04f

/* ======================== 功能开关 ======================== */

#define ENABLE_MADGWICK         1       // 1=Madgwick, 0=互补滤波
#define ENABLE_ENCODER          0       // 编码器支持 (暂时关闭)
#define ENABLE_REMOTE           0       // 遥控支持 (暂时关闭)
#define ENABLE_LOGGING          1       // 数据日志
#define ENABLE_STATUS_LED       0       // 状态指示灯

/* 日志文件路径 */
#define LOG_FILE_PATH           "/var/log/pacer/pacer.log"

#endif /* PACER_CONFIG_H */
