/**
 * @file led.h
 * @brief 状态指示灯模块
 *
 * LED 状态映射:
 *   常亮   = 已解锁 (armed)
 *   慢闪   = 地面 idle，等待解锁
 *   快闪   = 飞行中
 *   双闪   = 紧急/故障
 *   呼吸   = IMU 校准中
 */

#ifndef PACER_LED_H
#define PACER_LED_H

#include <stdbool.h>

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK_SLOW,     /* 地面 idle */
    LED_BLINK_FAST,     /* 飞行中 */
    LED_BLINK_DOUBLE,   /* 故障 */
    LED_BREATH,         /* 校准中 */
} led_state_t;

void led_init(void);
void led_set_state(led_state_t state);
void led_tick(void);    /* 在 10Hz 任务中调用 */

#endif /* PACER_LED_H */
