/**
 * @file quad_mixer.h
 * @brief 四旋翼混控器
 *
 * 将姿态控制器输出 (throttle, roll, pitch, yaw) 混合为四个电机油门。
 *
 * 电机布局 ("X" 型):
 *
 *     前
 *    FL  FR      FL=前左(CW)   FR=前右(CCW)
 *     \/
 *     /\
 *    RL  RR      RL=后左(CCW)  RR=后右(CW)
 *     后
 *
 * 混控公式:
 *   FL = throttle - pitch + roll - yaw
 *   FR = throttle - pitch - roll + yaw
 *   RL = throttle + pitch + roll + yaw
 *   RR = throttle + pitch - roll - yaw
 *
 * roll 正 = 右倾 → 右侧减速, 左侧加速
 * pitch 正 = 前倾 → 前侧减速, 后侧加速
 * yaw 正 = 顺时针 → CW 加速, CCW 减速
 */

#ifndef PACER_QUAD_MIXER_H
#define PACER_QUAD_MIXER_H

#include <stdint.h>
#include <stdbool.h>

#define QUAD_NUM_MOTORS  4

/* 电机索引 */
enum {
    QUAD_FL = 0,   /* 前左 — 顺时针 (CW) */
    QUAD_FR = 1,   /* 前右 — 逆时针 (CCW) */
    QUAD_RL = 2,   /* 后左 — 逆时针 (CCW) */
    QUAD_RR = 3,   /* 后右 — 顺时针 (CW) */
};

/* 混控器输出 */
typedef struct {
    float motor[QUAD_NUM_MOTORS];   /* 0.0 ~ 1.0 油门比例 */
} mixer_output_t;

/* 混控器配置 */
typedef struct {
    /* 油门范围限制 */
    float throttle_min;     /* 最低油门 (保持转动), 默认 0.05 */
    float throttle_max;     /* 最高油门, 默认 1.0 */
    float throttle_hover;   /* 悬停油门 (0~1), 默认 0.5 */

    /* 姿态限制 (混控前的输出限幅) */
    float max_roll_rate;    /* 最大 roll 校正量 */
    float max_pitch_rate;   /* 最大 pitch 校正量 */
    float max_yaw_rate;     /* 最大 yaw 校正量 */
} mixer_config_t;

#define MIXER_CONFIG_DEFAULT { \
    .throttle_min   = 0.05f,  \
    .throttle_max   = 1.0f,   \
    .throttle_hover = 0.5f,   \
    .max_roll_rate  = 0.5f,   \
    .max_pitch_rate = 0.5f,   \
    .max_yaw_rate   = 0.3f,   \
}

/* 混控器状态 */
typedef struct {
    mixer_config_t cfg;
    bool  armed;
    bool  enabled;
} quad_mixer_t;

/* ---------- 生命周期 ---------- */

/**
 * @brief 初始化混控器
 */
int  quad_mixer_init(quad_mixer_t *m, const mixer_config_t *cfg);

/**
 * @brief 解锁电机 (发送最小油门)
 */
void quad_mixer_arm(quad_mixer_t *m);

/**
 * @brief 上锁 (油门归零)
 */
void quad_mixer_disarm(quad_mixer_t *m);

/* ---------- 计算 ---------- */

/**
 * @brief 混控计算
 * @param m        混控器
 * @param throttle 油门 0~1 (总距)
 * @param roll     roll 校正 -1~+1 (姿态PID输出)
 * @param pitch    pitch 校正 -1~+1
 * @param yaw      yaw 校正 -1~+1
 * @return         四个电机的油门
 */
mixer_output_t quad_mixer_update(quad_mixer_t *m,
                                  float throttle,
                                  float roll,
                                  float pitch,
                                  float yaw);

/**
 * @brief 全部归零 (急停)
 */
void quad_mixer_stop(quad_mixer_t *m);

/**
 * @brief 清理
 */
void quad_mixer_deinit(quad_mixer_t *m);

/* ---------- 状态查询 ---------- */

bool quad_mixer_is_armed(const quad_mixer_t *m);

#endif /* PACER_QUAD_MIXER_H */
