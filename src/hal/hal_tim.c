/**
 * @file hal_tim.c
 * @brief 硬件 Timer PWM 实现 — STM32H743 TIM1
 *
 * 4 通道 ESC 输出: PA8(CH1), PE11(CH2), PE13(CH3), PE14(CH4)
 * 50Hz, 1000~2000μs 脉宽
 *
 * 时钟链:
 *   PCLK2 64MHz → TIM1 内核 128MHz
 *   预分频 128-1 → 计数频率 1MHz (1μs/tick)
 *   ARR 20000-1 → 周期 20ms = 50Hz
 */

#include "hal/hal_tim.h"
#include "app/config.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* TIM1 全局句柄 */
TIM_HandleTypeDef htim1;

/* 通道映射 */
static const uint32_t esc_channels[HAL_ESC_CH_COUNT] = {
    TIM_CHANNEL_1,  /* FL */
    TIM_CHANNEL_2,  /* FR */
    TIM_CHANNEL_3,  /* RL */
    TIM_CHANNEL_4,  /* RR */
};

/* 当前脉宽缓存 */
static uint16_t esc_pulse[HAL_ESC_CH_COUNT];

/* PWM 是否在运行 */
static bool pwm_running = false;

/* ---- GPIO 初始化 (TIM1 CH1~4 引脚) ---- */

static void tim1_gpio_init(void)
{
    /* PA8  — TIM1_CH1 (FL) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &g);

    /* PE11 — TIM1_CH2 (FR)
     * PE13 — TIM1_CH3 (RL)
     * PE14 — TIM1_CH4 (RR) */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOE, &g);
}

/* ---- TIM1 基础配置 ---- */

static void tim1_base_config(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = CFG_ESC_PWM_PRESCALER;    /* 128-1 → 1MHz */
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = CFG_ESC_PWM_PERIOD - 1;   /* 20000-1 → 50Hz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&htim1);
}

/* ---- PWM 通道配置 ---- */

static void tim1_channel_config(void)
{
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState= TIM_OCNIDLESTATE_RESET;

    /* 所有通道初始 1000μs = ESC 停止 */
    oc.Pulse = CFG_ESC_PULSE_MIN;

    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        HAL_TIM_PWM_ConfigChannel(&htim1, &oc, esc_channels[i]);
        esc_pulse[i] = CFG_ESC_PULSE_MIN;
    }
}

/* ---- 公开接口 ---- */

int hal_tim_pwm_init(void)
{
    tim1_gpio_init();
    tim1_base_config();
    tim1_channel_config();

    /* 主输出使能 (TIM1 高级定时器需要) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* MOE (Main Output Enable) — TIM1 特有 */
    __HAL_TIM_MOE_ENABLE(&htim1);

    pwm_running = true;
    return 0;
}

void hal_tim_pwm_set_pulse(int channel, uint16_t pulse_us)
{
    if (channel < 0 || channel >= HAL_ESC_CH_COUNT) return;
    if (pulse_us < CFG_ESC_PULSE_MIN) pulse_us = CFG_ESC_PULSE_MIN;
    if (pulse_us > CFG_ESC_PULSE_MAX) pulse_us = CFG_ESC_PULSE_MAX;

    esc_pulse[channel] = pulse_us;

    /* 直接写 CCR 寄存器 */
    switch (esc_channels[channel]) {
    case TIM_CHANNEL_1: htim1.Instance->CCR1 = pulse_us; break;
    case TIM_CHANNEL_2: htim1.Instance->CCR2 = pulse_us; break;
    case TIM_CHANNEL_3: htim1.Instance->CCR3 = pulse_us; break;
    case TIM_CHANNEL_4: htim1.Instance->CCR4 = pulse_us; break;
    }
}

void hal_tim_pwm_stop_all(void)
{
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        HAL_TIM_PWM_Stop(&htim1, esc_channels[i]);
    }
    __HAL_TIM_MOE_DISABLE(&htim1);
    pwm_running = false;
}

void hal_tim_pwm_start_all(void)
{
    for (int i = 0; i < HAL_ESC_CH_COUNT; i++) {
        HAL_TIM_PWM_Start(&htim1, esc_channels[i]);
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
    pwm_running = true;
}
