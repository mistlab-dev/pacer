/**
 * @file lift.c
 * @brief 升降机构实现 (骨架)
 *
 * 当前是框架代码，等硬件到位后填充实际控制逻辑。
 * 预留了 4 轮独立升降 + 状态机的完整接口。
 */

#include "lift/lift.h"
#include "hal/hal_pca9685.h"

#include <stdio.h>
#include <string.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int lift_init(lift_t *l, const lift_config_t *cfg)
{
    memset(l, 0, sizeof(*l));
    l->cfg   = *cfg;
    l->state = LIFT_STOWED;
    l->ready = true;

    /* 所有升降通道归零 */
    for (int i = 0; i < 4; i++) {
        l->height_cm[i] = 0.0f;
    }

    printf("[LIFT] init ch=%d,%d,%d,%d max=%.1fcm\n",
           cfg->ch_fl, cfg->ch_fr, cfg->ch_rl, cfg->ch_rr,
           cfg->max_height_cm);
    return 0;
}

void lift_deploy(lift_t *l)
{
    if (l->state != LIFT_STOWED) return;
    l->state = LIFT_DEPLOYING;
    printf("[LIFT] deploying...\n");
    /* TODO: 发送展开信号到舵机/电机 */
    l->state = LIFT_DEPLOYED;
    printf("[LIFT] deployed\n");
}

void lift_raise(lift_t *l, int wheel, float height)
{
    if (l->state != LIFT_DEPLOYED && l->state != LIFT_RAISED) return;

    height = clampf(height, 0.0f, l->cfg.max_height_cm);

    if (wheel >= 0 && wheel < 4) {
        l->height_cm[wheel] = height;
    } else {
        for (int i = 0; i < 4; i++) {
            l->height_cm[i] = height;
        }
    }

    l->state = LIFT_RAISING;
    printf("[LIFT] raising wheel=%d height=%.1fcm\n", wheel, height);

    /* TODO: 将 height_cm 映射为 PCA9685 PWM duty */
    /* uint16_t duty = height_to_duty(height); */
    /* hal_pca9685_set_pwm(ch, duty, 4095); */

    l->state = LIFT_RAISED;
}

void lift_lower_all(lift_t *l)
{
    for (int i = 0; i < 4; i++) {
        l->height_cm[i] = 0.0f;
    }
    l->state = LIFT_LOWERING;
    printf("[LIFT] lowering all\n");
    /* TODO: 降下逻辑 */
    l->state = LIFT_DEPLOYED;
}

void lift_stow(lift_t *l)
{
    lift_lower_all(l);
    l->state = LIFT_STOWED;
    printf("[LIFT] stowed\n");
}

void lift_update(lift_t *l, float dt)
{
    /* TODO: 实际运动控制 — 检测到位、平滑过渡等 */
    (void)l;
    (void)dt;
}

lift_state_t lift_get_state(const lift_t *l)
{
    return l->state;
}

void lift_deinit(lift_t *l)
{
    lift_stow(l);
    l->ready = false;
    printf("[LIFT] deinit\n");
}
