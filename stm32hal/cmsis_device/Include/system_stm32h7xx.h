/**
 * @file system_stm32h7xx.h
 * @brief STM32H7 系统配置头文件
 */

#ifndef SYSTEM_STM32H7XX_H
#define SYSTEM_STM32H7XX_H

#include <stdint.h>

extern uint32_t SystemCoreClock;

void SystemInit(void);
void SystemCoreClockUpdate(void);

#endif /* SYSTEM_STM32H7XX_H */