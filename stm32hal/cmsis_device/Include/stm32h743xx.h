/**
 * @file stm32h743xx.h
 * @brief STM32H743VI 寄存器定义 — 精简版
 *
 * 只包含飞控需要的寄存器：RCC, GPIO, I2C, TIM, UART
 */

#ifndef STM32H743XX_H
#define STM32H743XX_H

#include <stdint.h>

/* ============ 基地址定义 ============ */
#define PERIPH_BASE         0x40000000UL
#define D1_APB1PERIPH_BASE  PERIPH_BASE
#define D1_APB2PERIPH_BASE  (PERIPH_BASE + 0x00010000UL)
#define D1_AHB1PERIPH_BASE  (PERIPH_BASE + 0x00100000UL)
#define D1_AHB2PERIPH_BASE  (PERIPH_BASE + 0x00200000UL)

#define AHB1PERIPH_BASE     D1_AHB1PERIPH_BASE
#define APB1PERIPH_BASE     D1_APB1PERIPH_BASE
#define APB2PERIPH_BASE     D1_APB2PERIPH_BASE

/* GPIO 基地址 */
#define GPIOA_BASE          (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE          (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE          (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE          (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE          (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE          (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE          (AHB1PERIPH_BASE + 0x1C00UL)

/* RCC 基地址 */
#define RCC_BASE            (D1_AHB1PERIPH_BASE + 0x4400UL)

/* I2C 基地址 */
#define I2C1_BASE           (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE           (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE           (APB1PERIPH_BASE + 0x5C00UL)
#define I2C4_BASE           (APB4PERIPH_BASE + 0x6000UL)

/* Timer 基地址 */
#define TIM1_BASE           (APB2PERIPH_BASE + 0x0000UL)
#define TIM2_BASE           (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE           (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE           (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE           (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE           (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE           (APB1PERIPH_BASE + 0x1400UL)
#define TIM8_BASE           (APB2PERIPH_BASE + 0x0400UL)

/* UART 基地址 */
#define USART1_BASE         (APB2PERIPH_BASE + 0x1000UL)
#define USART2_BASE         (APB1PERIPH_BASE + 0x2400UL)
#define USART3_BASE         (APB1PERIPH_BASE + 0x2800UL)
#define UART4_BASE          (APB1PERIPH_BASE + 0x2C00UL)
#define UART5_BASE          (APB1PERIPH_BASE + 0x3000UL)
#define USART6_BASE         (APB2PERIPH_BASE + 0x1400UL)

/* Flash 基地址 */
#define FLASH_R_BASE        (AHB1PERIPH_BASE + 0x2000UL)
#define FLASH_BANK1_BASE    0x08000000UL
#define FLASH_BANK2_BASE    0x08100000UL

/* ============ 寄存器结构体定义 ============ */

typedef struct {
  volatile uint32_t MODER;      /* GPIO mode register */
  volatile uint32_t OTYPER;     /* GPIO output type register */
  volatile uint32_t OSPEEDR;    /* GPIO output speed register */
  volatile uint32_t PUPDR;      /* GPIO pull-up/pull-down register */
  volatile uint32_t IDR;        /* GPIO input data register */
  volatile uint32_t ODR;        /* GPIO output data register */
  volatile uint32_t BSRR;       /* GPIO bit set/reset register */
  volatile uint32_t LCKR;       /* GPIO lock register */
  volatile uint32_t AFR[2];     /* GPIO alternate function registers */
} GPIO_TypeDef;

typedef struct {
  volatile uint32_t CR;         /* RCC clock control register */
  volatile uint32_t HSICFGR;    /* HSI clock configuration register */
  volatile uint32_t CRRCR;      /* RCC CRS clock control register */
  volatile uint32_t CSICFGR;    /* CSI clock configuration register */
  volatile uint32_t CFGR;       /* RCC clock configuration register */
  volatile uint32_t D1CFGR;     /* RCC Domain 1 configuration register */
  volatile uint32_t D2CFGR;     /* RCC Domain 2 configuration register */
  volatile uint32_t D3CFGR;     /* RCC Domain 3 configuration register */
  volatile uint32_t RESERVED0;
  volatile uint32_t PLLCKSELR;  /* RCC PLL clock source selection register */
  volatile uint32_t PLLCFGR;    /* RCC PLL configuration register */
  volatile uint32_t PLL1DIVR;   /* RCC PLL1 dividers configuration register */
  volatile uint32_t PLL1FRACR;  /* RCC PLL1 fractional divider register */
  volatile uint32_t PLL2DIVR;   /* RCC PLL2 dividers configuration register */
  volatile uint32_t PLL2FRACR;  /* RCC PLL2 fractional divider register */
  volatile uint32_t PLL3DIVR;   /* RCC PLL3 dividers configuration register */
  volatile uint32_t PLL3FRACR;  /* RCC PLL3 fractional divider register */
  volatile uint32_t RESERVED1;
  volatile uint32_t D1CCIPR;    /* RCC Domain 1 kernel clock configuration register */
  volatile uint32_t D2CCIP1R;   /* RCC Domain 2 kernel clock configuration register 1 */
  volatile uint32_t D2CCIP2R;   /* RCC Domain 2 kernel clock configuration register 2 */
  volatile uint32_t D3CCIPR;    /* RCC Domain 3 kernel clock configuration register */
  volatile uint32_t RESERVED2;
  volatile uint32_t CIER;       /* RCC clock interrupt enable register */
  volatile uint32_t CIFR;       /* RCC clock interrupt flag register */
  volatile uint32_t CICR;       /* RCC clock interrupt clear register */
  volatile uint32_t RESERVED3;
  volatile uint32_t BDCR;       /* RCC Backup domain control register */
  volatile uint32_t CSR;        /* RCC clock control & status register */
  volatile uint32_t RESERVED4[2];
  volatile uint32_t AHB3RSTR;   /* RCC AHB3 peripheral reset register */
  volatile uint32_t AHB1RSTR;   /* RCC AHB1 peripheral reset register */
  volatile uint32_t AHB2RSTR;   /* RCC AHB2 peripheral reset register */
  volatile uint32_t AHB4RSTR;   /* RCC AHB4 peripheral reset register */
  volatile uint32_t APB3RSTR;   /* RCC APB3 peripheral reset register */
  volatile uint32_t APB1LRSTR;  /* RCC APB1L peripheral reset register */
  volatile uint32_t APB1HRSTR;  /* RCC APB1H peripheral reset register */
  volatile uint32_t APB2RSTR;   /* RCC APB2 peripheral reset register */
  volatile uint32_t APB4RSTR;   /* RCC APB4 peripheral reset register */
  volatile uint32_t RESERVED5;
  volatile uint32_t AHB3ENR;    /* RCC AHB3 peripheral clock enable register */
  volatile uint32_t AHB1ENR;    /* RCC AHB1 peripheral clock enable register */
  volatile uint32_t AHB2ENR;    /* RCC AHB2 peripheral clock enable register */
  volatile uint32_t AHB4ENR;    /* RCC AHB4 peripheral clock enable register */
  volatile uint32_t APB3ENR;    /* RCC APB3 peripheral clock enable register */
  volatile uint32_t APB1LENR;   /* RCC APB1L peripheral clock enable register */
  volatile uint32_t APB1HENR;   /* RCC APB1H peripheral clock enable register */
  volatile uint32_t APB2ENR;    /* RCC APB2 peripheral clock enable register */
  volatile uint32_t APB4ENR;    /* RCC APB4 peripheral clock enable register */
} RCC_TypeDef;

typedef struct {
  volatile uint32_t CR1;        /* I2C control register 1 */
  volatile uint32_t CR2;        /* I2C control register 2 */
  volatile uint32_t OAR1;       /* I2C own address register 1 */
  volatile uint32_t OAR2;       /* I2C own address register 2 */
  volatile uint32_t TIMINGR;    /* I2C timing register */
  volatile uint32_t TIMEOUTR;   /* I2C timeout register */
  volatile uint32_t ISR;        /* I2C interrupt and status register */
  volatile uint32_t ICR;        /* I2C interrupt clear register */
  volatile uint32_t PECR;       /* I2C PEC register */
  volatile uint32_t RXDR;       /* I2C reception data register */
  volatile uint32_t TXDR;       /* I2C transmission data register */
} I2C_TypeDef;

typedef struct {
  volatile uint32_t CR1;        /* TIM control register 1 */
  volatile uint32_t CR2;        /* TIM control register 2 */
  volatile uint32_t SMCR;       /* TIM slave mode control register */
  volatile uint32_t DIER;       /* TIM DMA/interrupt enable register */
  volatile uint32_t SR;         /* TIM status register */
  volatile uint32_t EGR;        /* TIM event generation register */
  volatile uint32_t CCMR1;      /* TIM capture/compare mode register 1 */
  volatile uint32_t CCMR2;      /* TIM capture/compare mode register 2 */
  volatile uint32_t CCER;       /* TIM capture/compare enable register */
  volatile uint32_t CNT;        /* TIM counter register */
  volatile uint32_t PSC;        /* TIM prescaler register */
  volatile uint32_t ARR;        /* TIM auto-reload register */
  volatile uint32_t RCR;        /* TIM repetition counter register */
  volatile uint32_t CCR1;       /* TIM capture/compare register 1 */
  volatile uint32_t CCR2;       /* TIM capture/compare register 2 */
  volatile uint32_t CCR3;       /* TIM capture/compare register 3 */
  volatile uint32_t CCR4;       /* TIM capture/compare register 4 */
  volatile uint32_t BDTR;       /* TIM break and dead-time register */
  volatile uint32_t DCR;        /* TIM DMA control register */
  volatile uint32_t DMAR;       /* TIM DMA address register */
  volatile uint32_t RESERVED;
  volatile uint32_t CCMR3;      /* TIM capture/compare mode register 3 */
} TIM_TypeDef;

typedef struct {
  volatile uint32_t CR1;        /* USART control register 1 */
  volatile uint32_t CR2;        /* USART control register 2 */
  volatile uint32_t CR3;        /* USART control register 3 */
  volatile uint32_t BRR;        /* USART baud rate register */
  volatile uint32_t GTPR;       /* USART guard time and prescaler register */
  volatile uint32_t RTOR;       /* USART receiver timeout register */
  volatile uint32_t ISR;        /* USART interrupt and status register */
  volatile uint32_t ICR;        /* USART interrupt flag clear register */
  volatile uint32_t RDR;        /* USART receive data register */
  volatile uint32_t TDR;        /* USART transmit data register */
} USART_TypeDef;

typedef struct {
  volatile uint32_t ACR;        /* Flash access control register */
  volatile uint32_t RESERVED0;
  volatile uint32_t KEYR;       /* Flash key register */
  volatile uint32_t OPTKEYR;    /* Flash option key register */
  volatile uint32_t CR;         /* Flash control register */
  volatile uint32_t SR;         /* Flash status register */
  volatile uint32_t CCR;        /* Flash clear control register */
  volatile uint32_t RESERVED1;
  volatile uint32_t OPTCR;      /* Flash option control register */
  volatile uint32_t OPTCR2;     /* Flash option control register 2 */
} FLASH_TypeDef;

/* ============ 寄存器位定义 ============ */

/* GPIO MODER */
#define GPIO_MODE_INPUT           0x00U
#define GPIO_MODE_OUTPUT          0x01U
#define GPIO_MODE_AF              0x02U
#define GPIO_MODE_ANALOG          0x03U

/* GPIO OTYPER */
#define GPIO_OTYPE_PP             0x00U
#define GPIO_OTYPE_OD             0x01U

/* GPIO OSPEEDR */
#define GPIO_SPEED_FREQ_LOW       0x00U
#define GPIO_SPEED_FREQ_MEDIUM    0x01U
#define GPIO_SPEED_FREQ_HIGH      0x02U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x03U

/* GPIO PUPDR */
#define GPIO_NOPULL               0x00U
#define GPIO_PULLUP               0x01U
#define GPIO_PULLDOWN             0x02U

/* GPIO AFR */
#define GPIO_AF0                  0x00U
#define GPIO_AF4_I2C1             0x04U
#define GPIO_AF7_USART1           0x07U
#define GPIO_AF7_USART2           0x07U
#define GPIO_AF7_USART3           0x07U

/* RCC CR */
#define RCC_CR_HSION              (1U << 0)
#define RCC_CR_HSIRDY             (1U << 1)
#define RCC_CR_HSEON              (1U << 8)
#define RCC_CR_HSERDY             (1U << 9)
#define RCC_CR_PLLON              (1U << 24)
#define RCC_CR_PLLRDY             (1U << 25)

/* RCC CFGR */
#define RCC_CFGR_SW_HSI           0x00U
#define RCC_CFGR_SW_HSE           0x01U
#define RCC_CFGR_SW_PLL1          0x02U
#define RCC_CFGR_SW_PLL2          0x03U
#define RCC_CFGR_SWS              (0x03U << 3)
#define RCC_CFGR_SWS_Pos          3U

/* RCC PLLCKSELR */
#define RCC_PLLCKSELR_PLLSRC_HSI  0x00U
#define RCC_PLLCKSELR_PLLSRC_HSE  0x01U
#define RCC_PLLCKSELR_PLLSRC_Pos  0U
#define RCC_PLLCKSELR_DIVM1_Pos   4U
#define RCC_PLLCKSELR_DIVM1       (0x3FU << RCC_PLLCKSELR_DIVM1_Pos)

/* RCC PLL1DIVR */
#define RCC_PLL1DIVR_N1_Pos       0U
#define RCC_PLL1DIVR_N1           (0x1FFU << RCC_PLL1DIVR_N1_Pos)
#define RCC_PLL1DIVR_P1_Pos       9U
#define RCC_PLL1DIVR_P1           (0x1FU << RCC_PLL1DIVR_P1_Pos)

/* RCC AHB1ENR */
#define RCC_AHB1ENR_GPIOAEN       (1U << 0)
#define RCC_AHB1ENR_GPIOBEN       (1U << 1)
#define RCC_AHB1ENR_GPIOCEN       (1U << 2)
#define RCC_AHB1ENR_GPIODEN       (1U << 3)
#define RCC_AHB1ENR_GPIOEEN       (1U << 4)

/* RCC APB1LENR */
#define RCC_APB1LENR_I2C1EN       (1U << 12)
#define RCC_APB1LENR_USART2EN     (1U << 17)
#define RCC_APB1LENR_USART3EN     (1U << 18)

/* RCC APB2ENR */
#define RCC_APB2ENR_TIM1EN        (1U << 0)
#define RCC_APB2ENR_USART1EN      (1U << 4)
#define RCC_APB2ENR_USART6EN      (1U << 5)

/* I2C CR1 */
#define I2C_CR1_PE                (1U << 0)
#define I2C_CR1_TXIE              (1U << 1)
#define I2C_CR1_RXIE              (1U << 2)

/* I2C ISR */
#define I2C_ISR_TXE               (1U << 0)
#define I2C_ISR_RXNE              (1U << 2)
#define I2C_ISR_TC                (1U << 6)

/* I2C CR2 */
#define I2C_CR2_START             (1U << 13)
#define I2C_CR2_STOP              (1U << 14)
#define I2C_CR2_NBYTES_Pos        24U

/* TIM CR1 */
#define TIM_CR1_CEN               (1U << 0)
#define TIM_CR1_ARPE              (1U << 7)

/* TIM CCER */
#define TIM_CCER_CC1E             (1U << 0)
#define TIM_CCER_CC2E             (1U << 4)
#define TIM_CCER_CC3E             (1U << 8)
#define TIM_CCER_CC4E             (1U << 12)

/* TIM CCMR1 (PWM mode) */
#define TIM_CCMR1_OC1M_PWM1       (0x06U << 4)
#define TIM_CCMR1_OC1PE           (1U << 3)

/* TIM BDTR */
#define TIM_BDTR_MOE              (1U << 15)

/* USART CR1 */
#define USART_CR1_TE              (1U << 3)
#define USART_CR1_RE              (1U << 2)
#define USART_CR1_UE              (1U << 0)

/* USART ISR */
#define USART_ISR_TXE             (1U << 7)
#define USART_ISR_RXNE            (1U << 5)
#define USART_ISR_TC              (1U << 6)

/* Flash ACR */
#define FLASH_ACR_LATENCY_0WS     0x00U
#define FLASH_ACR_LATENCY_1WS     0x01U
#define FLASH_ACR_LATENCY_2WS     0x02U
#define FLASH_ACR_LATENCY_3WS     0x03U

/* ============ 外设指针声明 ============ */

#define GPIOA                     ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                     ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                     ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD                     ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE                     ((GPIO_TypeDef *) GPIOE_BASE)

#define RCC                       ((RCC_TypeDef *) RCC_BASE)

#define I2C1                      ((I2C_TypeDef *) I2C1_BASE)

#define TIM1                      ((TIM_TypeDef *) TIM1_BASE)
#define TIM2                      ((TIM_TypeDef *) TIM2_BASE)

#define USART1                    ((USART_TypeDef *) USART1_BASE)
#define USART2                    ((USART_TypeDef *) USART2_BASE)
#define USART3                    ((USART_TypeDef *) USART3_BASE)

#define FLASH                     ((FLASH_TypeDef *) FLASH_R_BASE)

#endif /* STM32H743XX_H */