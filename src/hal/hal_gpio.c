/**
 * @file hal_gpio.c
 * @brief GPIO/延时 硬件抽象 — STM32H743 实现
 *
 * GPIO 操作基于 STM32 HAL。
 * 微秒延时和微秒时间戳基于 DWT (Data Watchpoint and Trace) 计数器。
 */

#include "hal/hal_gpio.h"
#include "stm32h7xx_hal.h"

/* DWT 计数器 — 用于精确微秒计时 */
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DEM_CR      (*(volatile uint32_t *)0xE000EDFC)

static int g_initialized = 0;

/* 64-bit 微秒计数器 — 滚动补偿 */
static uint32_t g_last_cycles = 0;
static uint64_t g_micros_base = 0;

int hal_gpio_init(void)
{
    if (g_initialized) return 0;

    /* 使能 DWT 计数器 (用于精确延时) */
    DEM_CR |= (1 << 24);           /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1;                 /* CYCCNTENA */

    g_initialized = 1;
    return 0;
}

void hal_gpio_deinit(void)
{
    /* STM32 GPIO 由 HAL 管理，不需要手动 deinit */
    g_initialized = 0;
}

/* ---- 引脚编号转换 ---- */

static void pin_to_port(int pin, GPIO_TypeDef **port, uint16_t *mask)
{
    static GPIO_TypeDef *ports[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
        GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK
    };
    int p = pin / 16;
    int n = pin % 16;

    if (p > 10) p = 0;
    *port = ports[p];
    *mask = (uint16_t)(1U << n);
}

void hal_gpio_set_mode(int pin, int mode)
{
    GPIO_TypeDef *port;
    uint16_t mask;
    pin_to_port(pin, &port, &mask);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = mask;
    gpio.Mode  = mode ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

void hal_gpio_write(int pin, int value)
{
    GPIO_TypeDef *port;
    uint16_t mask;
    pin_to_port(pin, &port, &mask);
    HAL_GPIO_WritePin(port, mask, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int hal_gpio_read(int pin)
{
    GPIO_TypeDef *port;
    uint16_t mask;
    pin_to_port(pin, &port, &mask);
    return (HAL_GPIO_ReadPin(port, mask) == GPIO_PIN_SET) ? 1 : 0;
}

/* ---- 精确延时 (DWT) ---- */

void hal_delay_us(unsigned int us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    uint32_t start  = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles) {
        __NOP();
    }
}

void hal_delay_ms(unsigned int ms)
{
    HAL_Delay(ms);
}

unsigned int hal_micros(void)
{
    return (unsigned int)hal_micros_64();
}

uint64_t hal_micros_64(void)
{
    uint32_t now = DWT_CYCCNT;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    /* 检测 DWT 32-bit 回卷 */
    if (now < g_last_cycles) {
        g_micros_base += 0x100000000ULL / cycles_per_us;
    }
    g_last_cycles = now;

    return g_micros_base + (uint64_t)(now / cycles_per_us);
}
