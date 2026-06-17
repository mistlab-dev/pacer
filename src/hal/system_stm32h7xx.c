/**
 * @file system_stm32h7xx.c
 * @brief STM32H743 系统初始化 - CMSIS 级
 *
 * 提供 SystemInit() 和 SystemCoreClock 变量。
 */

#include "stm32h743xx.h"

uint32_t SystemCoreClock = 64000000UL; /* 64MHz 默认（HSI） */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location.
  * @note   This function is called at startup before Reset_Handler.
  */
void SystemInit(void)
{
  /* FPU 设置 */
  #if defined (__FPU_PRESENT) && (__FPU_PRESENT == 1U)
    SCB->CPACR |= ((3U << 10U*2U) |         /* CP10 Full Access */
                   (3U << 11U*2U)  );       /* CP11 Full Access */
  #endif

  /* 复位 RCC 时钟配置为默认状态 */
  RCC->CR |= RCC_CR_HSION;                  /* 使能 HSI */
  RCC->CFGR = 0x00000000U;                  /* Reset CFGR */

  /* 等待 HSI 就绪 */
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
  {
    /* Wait */
  }

  /* 关闭 HSE、PLL、PLL2、PLL3 */
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_PLLON |
               RCC_CR_PLL2ON | RCC_CR_PLL3ON);

  /* 等待 PLL 等关闭 */
  while ((RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLL2RDY | RCC_CR_PLL3RDY)) != 0U)
  {
    /* Wait */
  }

  /* 复位 PLL 配置 */
  RCC->PLLCFGR = 0x00000000U;

  /* 复位中断和清除中断标志 */
  RCC->CIER = 0x00000000U;
  RCC->CICR = 0xFFFFFFFFU;

  /* 复位 PLL1 分频 */
  RCC->PLL1DIVR = 0x00000000U;

  /* 复位 PLL1 FRACN */
  RCC->PLL1FRACR = 0x00000000U;

  /* 复位 PLL2 分频 */
  RCC->PLL2DIVR = 0x00000000U;
  RCC->PLL2FRACR = 0x00000000U;

  /* 复位 PLL3 分频 */
  RCC->PLL3DIVR = 0x00000000U;
  RCC->PLL3FRACR = 0x00000000U;

  /* 复位 D1CFGR, D2CFGR, D3CFGR */
  RCC->D1CFGR = 0x00000000U;
  RCC->D2CFGR = 0x00000000U;
  RCC->D3CFGR = 0x00000000U;

  /* 复位 Flash 延迟 */
  FLASH->ACR = FLASH_ACR_LATENCY_0WS;

  /* 设置向量表位置（Flash 基址） */
  SCB->VTOR = FLASH_BANK1_BASE;

  /* 临时设置 SystemCoreClock 为 HSI (64MHz) */
  SystemCoreClock = 64000000UL;
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock frequency.
  */
void SystemCoreClockUpdate(void)
{
  uint32_t hsi = 64000000UL;
  uint32_t hse = 8000000UL;
  uint32_t pll_source;
  uint32_t pll_m, pll_n, pll_p;
  uint32_t sysclk_source;

  /* Get SYSCLK source */
  sysclk_source = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;

  switch (sysclk_source)
  {
    case 0x00U:  /* HSI used as system clock source */
      SystemCoreClock = hsi;
      break;

    case 0x01U:  /* HSE used as system clock source */
      SystemCoreClock = hse;
      break;

    case 0x02U:  /* PLL1 used as system clock source */
      pll_source = (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC) >> RCC_PLLCKSELR_PLLSRC_Pos;
      pll_m = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos;
      pll_n = (RCC->PLL1DIVR & RCC_PLL1DIVR_N1) >> RCC_PLL1DIVR_N1_Pos;
      pll_p = (RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >> RCC_PLL1DIVR_P1_Pos;

      if (pll_source == 0x01U)  /* HSE */
      {
        SystemCoreClock = (hse / pll_m) * pll_n / pll_p;
      }
      else  /* HSI */
      {
        SystemCoreClock = (hsi / pll_m) * pll_n / pll_p;
      }
      break;

    case 0x03U:  /* PLL2 used as system clock source */
      /* Similar calculation for PLL2 */
      SystemCoreClock = 64000000UL;  /* Default */
      break;

    default:
      SystemCoreClock = hsi;
      break;
  }

  /* Apply D1 core clock prescaler */
  uint32_t d1cpre = (RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos;
  if (d1cpre >= 8U)
  {
    SystemCoreClock >>= (d1cpre - 7U);
  }
}