/**
 * @file led.c
 * @brief 状态指示灯实现 — STM32H743
 *
 * 使用板载 LED (PE1)，通过 hal_gpio 控制。
 * 在 10Hz 任务中调用 led_tick() 更新闪烁模式。
 */

#include "hal/led.h"
#include "hal/hal_gpio.h"
#include "app/config.h"

/* LED 引脚: PE1 = port_E(4) * 16 + 1 = 65 */
#define LED_PIN     65

static led_state_t g_state = LED_OFF;
static uint32_t   g_tick   = 0;

void led_init(void)
{
    hal_gpio_set_mode(LED_PIN, 1);  /* output */
    hal_gpio_write(LED_PIN, 0);
    g_state = LED_OFF;
    g_tick  = 0;
}

void led_set_state(led_state_t state)
{
    if (g_state != state) {
        g_state = state;
        g_tick  = 0;
    }
}

void led_tick(void)
{
    g_tick++;
    bool on = false;

    switch (g_state) {
    case LED_OFF:
        on = false;
        break;
    case LED_ON:
        on = true;
        break;
    case LED_BLINK_SLOW:
        /* 0.5Hz: 亮 1s 灭 1s (10 ticks = 1s) */
        on = (g_tick % 20) < 10;
        break;
    case LED_BLINK_FAST:
        /* 5Hz: 亮 100ms 灭 100ms */
        on = (g_tick % 2) == 0;
        break;
    case LED_BLINK_DOUBLE:
        /* 双闪: 亮-灭-亮-灭-灭-灭-灭 (周期 1s) */
        {
            uint32_t phase = g_tick % 10;
            on = (phase < 1) || (phase >= 2 && phase < 3);
        }
        break;
    case LED_BREATH:
        /* 呼吸灯: 用 PWM 模拟 (简单 4 级) */
        {
            uint32_t phase = g_tick % 20;
            if      (phase < 5)  on = true;
            else if (phase < 10) on = (g_tick % 2) == 0;
            else if (phase < 15) on = (g_tick % 3) == 0;
            else                 on = false;
        }
        break;
    }

    hal_gpio_write(LED_PIN, on ? 1 : 0);
}
