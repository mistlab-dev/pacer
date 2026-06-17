/**
 * @file battery.c
 * @brief 电池电压检测 — STM32H743 ADC 实现
 *
 * ADC1 通道 5 (PA5), 12-bit 分辨率
 * 参考电压 3.3V
 *
 * 分压网络:
 *   R1 = 33kΩ (上臂, 接电池+)
 *   R2 = 10kΩ (下臂, 接 GND)
 *   分压比 = R2 / (R1 + R2) = 0.2326
 *
 *   ADC_value = V_bat × 0.2326 / 3.3 × 4095
 *   V_bat    = ADC_value × 3.3 / 4095 / 0.2326
 */

#include "hal/battery.h"
#include "app/config.h"
#include "stm32h7xx_hal.h"

#define VREF            3.3f
#define ADC_RESOLUTION  4095.0f   /* 12-bit */
#define DIVIDER_RATIO   0.2326f   /* R2/(R1+R2) = 10/(33+10) */

/* 滤波: 滑动平均 */
#define BAT_FILTER_SIZE  8

static ADC_HandleTypeDef hadc1;
static float    g_voltage   = 0.0f;
static uint16_t g_filter_buf[BAT_FILTER_SIZE];
static uint8_t  g_filter_idx = 0;
static bool     g_ready      = false;

static float raw_to_voltage(uint16_t raw)
{
    return (float)raw * VREF / ADC_RESOLUTION / DIVIDER_RATIO;
}

void battery_init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 → ADC1_CH5 analog input */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) return;

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_5;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ch);

    /* 校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    g_ready = true;
}

void battery_update(void)
{
    if (!g_ready) return;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

        g_filter_buf[g_filter_idx] = raw;
        g_filter_idx = (g_filter_idx + 1) % BAT_FILTER_SIZE;

        /* 滑动平均 */
        uint32_t sum = 0;
        for (int i = 0; i < BAT_FILTER_SIZE; i++)
            sum += g_filter_buf[i];
        g_voltage = raw_to_voltage(sum / BAT_FILTER_SIZE);
    }
    HAL_ADC_Stop(&hadc1);
}

float battery_get_voltage(void)
{
    return g_voltage;
}

battery_status_t battery_get_status(void)
{
    if (g_voltage < CFG_BATTERY_CRITICAL_V) return BATTERY_CRITICAL;
    if (g_voltage < CFG_BATTERY_WARNING_V)  return BATTERY_WARNING;
    return BATTERY_OK;
}
