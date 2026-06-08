/**
 * @file diff_drive.h
 * @brief 四轮差速驱动控制器
 *
 * 4 个电机分为左右两组:
 *   左前 + 左后 = 左侧
 *   右前 + 右后 = 右侧
 *
 * 控制逻辑:
 *   left  = throttle - steering
 *   right = throttle + steering
 *
 * 支持速度环 PID (有编码器时) 或直接开环 PWM。
 */

#ifndef PACER_DIFF_DRIVE_H
#define PACER_DIFF_DRIVE_H

#include "motor/motor.h"
#include "ctrl/pid.h"

#define DIFF_DRIVE_NUM_MOTORS   4

/* 电机索引 */
enum {
    MOTOR_FL = 0,   /* 前左 */
    MOTOR_FR = 1,   /* 前右 */
    MOTOR_RL = 2,   /* 后左 */
    MOTOR_RR = 3,   /* 后右 */
};

typedef struct {
    motor_t motors[DIFF_DRIVE_NUM_MOTORS];

    /* 控制输入 */
    float throttle;     /* -1.0 ~ +1.0  前后 */
    float steering;     /* -1.0 ~ +1.0  左右 (正=右转) */

    /* 速度环 PID (可选, 有编码器时启用) */
    pacer_pid_t  pid_speed;
    bool   use_pid;     /* false=开环直接映射 */

    /* 安全限制 */
    float max_power;    /* 最大 PWM 占空比比例 (0~1.0) */
    bool  enabled;
} diff_drive_t;

/**
 * @brief 初始化差速驱动
 * @param dd     差速驱动
 * @param motors 4个电机的配置 (FL, FR, RL, RR)
 * @return 0=成功
 */
int  diff_drive_init(diff_drive_t *dd, motor_t motors[DIFF_DRIVE_NUM_MOTORS]);

/**
 * @brief 设置油门和转向
 * @param dd        差速驱动
 * @param throttle  -1.0(后退) ~ +1.0(前进)
 * @param steering  -1.0(左转) ~ +1.0(右转)
 */
void diff_drive_set(diff_drive_t *dd, float throttle, float steering);

/**
 * @brief 更新电机输出 (每控制周期调用)
 * @param dd  差速驱动
 * @param dt  时间步长 (秒)
 */
void diff_drive_update(diff_drive_t *dd, float dt);

/**
 * @brief 急停
 */
void diff_drive_brake(diff_drive_t *dd);

/**
 * @brief 使能/禁用
 */
void diff_drive_enable(diff_drive_t *dd, bool on);

/**
 * @brief 清理
 */
void diff_drive_deinit(diff_drive_t *dd);

#endif /* PACER_DIFF_DRIVE_H */
