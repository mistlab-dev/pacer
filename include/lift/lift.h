/**
 * @file lift.h
 * @brief 升降机构接口 (爬楼梯用)
 *
 * 每个轮子独立升降，通过 PCA9685 控制舵机/电机。
 * 状态机: 收起 → 展开 → 升起 → 楼梯攀爬
 *
 * PCA9685 通道分配:
 *   CH8  → 前左升降
 *   CH9  → 前右升降
 *   CH10 → 后左升降
 *   CH11 → 后右升降
 */

#ifndef PACER_LIFT_H
#define PACER_LIFT_H

#include <stdbool.h>
#include <stdint.h>

/* 升降状态 */
typedef enum {
    LIFT_STOWED,        /* 收起 (正常行驶) */
    LIFT_DEPLOYING,     /* 正在展开 */
    LIFT_DEPLOYED,      /* 已展开, 准备升起 */
    LIFT_RAISING,       /* 正在升起 */
    LIFT_RAISED,        /* 已升起到位 */
    LIFT_LOWERING,      /* 正在降下 */
    LIFT_ERROR,         /* 错误 */
} lift_state_t;

/* 升降配置 */
typedef struct {
    int ch_fl;          /* 前左 PCA9685 通道 */
    int ch_fr;          /* 前右 */
    int ch_rl;          /* 后左 */
    int ch_rr;          /* 后右 */
    float max_height_cm;
    float speed_cm_sec;  /* 升降速度 */
} lift_config_t;

#define LIFT_CONFIG_DEFAULT { \
    .ch_fl = 8, .ch_fr = 9, .ch_rl = 10, .ch_rr = 11, \
    .max_height_cm = 15.0f, \
    .speed_cm_sec = 3.0f, \
}

/* 升降状态 */
typedef struct {
    lift_config_t cfg;
    lift_state_t state;
    float height_cm[4];  /* 每个轮子的当前高度 */
    bool ready;
} lift_t;

/**
 * @brief 初始化升降机构
 */
int  lift_init(lift_t *l, const lift_config_t *cfg);

/**
 * @brief 展开升降机构 (从收起到就绪)
 */
void lift_deploy(lift_t *l);

/**
 * @brief 升起指定轮子
 * @param wheel  0=FL, 1=FR, 2=RL, 3=RR, -1=全部
 * @param height 目标高度 (cm)
 */
void lift_raise(lift_t *l, int wheel, float height);

/**
 * @brief 降下所有轮子
 */
void lift_lower_all(lift_t *l);

/**
 * @brief 收起升降机构
 */
void lift_stow(lift_t *l);

/**
 * @brief 更新升降状态 (在控制循环中调用)
 * @param dt 时间步长
 */
void lift_update(lift_t *l, float dt);

/**
 * @brief 获取当前状态
 */
lift_state_t lift_get_state(const lift_t *l);

/**
 * @brief 关闭
 */
void lift_deinit(lift_t *l);

#endif /* PACER_LIFT_H */
