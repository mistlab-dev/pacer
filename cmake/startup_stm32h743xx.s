/**
 * @file startup_stm32h743xx.s
 * @brief STM32H743VI 启动文件 - GCC ARM
 *
 * Vector table + reset handler for Cortex-M7
 */

  .syntax unified
  .cpu cortex-m7
  .fpu fpv5-sp-d16
  .thumb

.global  Reset_Handler
.global  Default_Handler

/* Vector table */
.section .isr_vector, "a", %progbits
.type   g_pfnVectors, %object
.size   g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word  _estack               /* Initial Stack Pointer */
  .word  Reset_Handler         /* Reset Handler */
  .word  NMI_Handler           /* NMI Handler */
  .word  HardFault_Handler     /* Hard Fault Handler */
  .word  MemManage_Handler     /* MPU Fault Handler */
  .word  BusFault_Handler      /* Bus Fault Handler */
  .word  UsageFault_Handler    /* Usage Fault Handler */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  0                     /* Reserved */
  .word  SVC_Handler           /* SVCall Handler */
  .word  DebugMon_Handler      /* Debug Monitor Handler */
  .word  0                     /* Reserved */
  .word  PendSV_Handler        /* PendSV Handler */
  .word  SysTick_Handler       /* SysTick Handler */

  /* External Interrupts */
  .word  WWDG_IRQHandler                   /* Window Watchdog */
  .word  PVD_AVD_IRQHandler                /* PVD/AVD */
  .word  TAMP_IRQHandler                   /* Tamper */
  .word  RTC_IRQHandler                    /* RTC */
  .word  FLASH_IRQHandler                  /* Flash */
  .word  RCC_IRQHandler                    /* RCC */
  .word  EXTI0_IRQHandler                  /* EXTI Line0 */
  .word  EXTI1_IRQHandler                  /* EXTI Line1 */
  .word  EXTI2_IRQHandler                  /* EXTI Line2 */
  .word  EXTI3_IRQHandler                  /* EXTI Line3 */
  .word  EXTI4_IRQHandler                  /* EXTI Line4 */
  .word  DMA1_Stream0_IRQHandler           /* DMA1 Stream0 */
  .word  DMA1_Stream1_IRQHandler           /* DMA1 Stream1 */
  .word  DMA1_Stream2_IRQHandler           /* DMA1 Stream2 */
  .word  DMA1_Stream3_IRQHandler           /* DMA1 Stream3 */
  .word  DMA1_Stream4_IRQHandler           /* DMA1 Stream4 */
  .word  DMA1_Stream5_IRQHandler           /* DMA1 Stream5 */
  .word  DMA1_Stream6_IRQHandler           /* DMA1 Stream6 */
  .word  ADC_IRQHandler                    /* ADC1/2 */
  .word  FDCAN1_IT0_IRQHandler             /* FDCAN1 IT0 */
  .word  FDCAN2_IT0_IRQHandler             /* FDCAN2 IT0 */
  .word  FDCAN1_IT1_IRQHandler             /* FDCAN1 IT1 */
  .word  FDCAN2_IT1_IRQHandler             /* FDCAN2 IT1 */
  .word  EXTI5_9_IRQHandler                 /* EXTI Line5..9 */
  .word  TIM1_BRK_IRQHandler               /* TIM1 Break */
  .word  TIM1_UP_IRQHandler                /* TIM1 Update */
  .word  TIM1_TRG_COM_IRQHandler           /* TIM1 TRG/COM */
  .word  TIM1_CC_IRQHandler                /* TIM1 Capture/Compare */
  .word  TIM2_IRQHandler                   /* TIM2 */
  .word  TIM3_IRQHandler                   /* TIM3 */
  .word  TIM4_IRQHandler                   /* TIM4 */
  .word  I2C1_EV_IRQHandler                /* I2C1 Event */
  .word  I2C1_ER_IRQHandler                /* I2C1 Error */
  .word  I2C2_EV_IRQHandler                /* I2C2 Event */
  .word  I2C2_ER_IRQHandler                /* I2C2 Error */
  .word  SPI1_IRQHandler                   /* SPI1 */
  .word  SPI2_IRQHandler                   /* SPI2 */
  .word  USART1_IRQHandler                 /* USART1 */
  .word  USART2_IRQHandler                 /* USART2 */
  .word  USART3_IRQHandler                 /* USART3 */
  .word  EXTI10_15_IRQHandler               /* EXTI Line10..15 */
  .word  RTC_Alarm_IRQHandler              /* RTC Alarm */
  .word  0                                 /* Reserved */
  .word  TIM8_BRK_IRQHandler               /* TIM8 Break */
  .word  TIM8_UP_IRQHandler                /* TIM8 Update */
  .word  TIM8_TRG_COM_IRQHandler           /* TIM8 TRG/COM */
  .word  TIM8_CC_IRQHandler                /* TIM8 CC */
  .word  DMA1_Stream7_IRQHandler           /* DMA1 Stream7 */
  .word  FMC_IRQHandler                    /* FMC */
  .word  SDMMC1_IRQHandler                 /* SDMMC1 */
  .word  TIM5_IRQHandler                   /* TIM5 */
  .word  SPI3_IRQHandler                   /* SPI3 */
  .word  UART4_IRQHandler                  /* UART4 */
  .word  UART5_IRQHandler                  /* UART5 */
  .word  TIM6_DAC_IRQHandler               /* TIM6/DAC */
  .word  TIM7_IRQHandler                   /* TIM7 */
  .word  DMA2_Stream0_IRQHandler           /* DMA2 Stream0 */
  .word  DMA2_Stream1_IRQHandler           /* DMA2 Stream1 */
  .word  DMA2_Stream2_IRQHandler           /* DMA2 Stream2 */
  .word  DMA2_Stream3_IRQHandler           /* DMA2 Stream3 */
  .word  DMA2_Stream4_IRQHandler           /* DMA2 Stream4 */
  .word  ETH_IRQHandler                    /* Ethernet */
  .word  ETH_WKUP_IRQHandler               /* ETH Wakeup */
  .word  FDCAN_CAL_IRQHandler              /* FDCAN Calibration */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  DMA2_Stream5_IRQHandler           /* DMA2 Stream5 */
  .word  DMA2_Stream6_IRQHandler           /* DMA2 Stream6 */
  .word  DMA2_Stream7_IRQHandler           /* DMA2 Stream7 */
  .word  USART6_IRQHandler                 /* USART6 */
  .word  I2C3_EV_IRQHandler                /* I2C3 Event */
  .word  I2C3_ER_IRQHandler                /* I2C3 Error */
  .word  USB_OTG1_EP1_OUT_IRQHandler       /* USB_OTG1 EP1 OUT */
  .word  USB_OTG1_EP1_IN_IRQHandler        /* USB_OTG1 EP1 IN */
  .word  USB_OTG1_WKUP_IRQHandler          /* USB_OTG1 Wakeup */
  .word  USB_OTG2_IRQHandler               /* USB_OTG2 */
  .word  DCMI_IRQHandler                   /* DCMI */
  .word  0                                 /* Reserved */
  .word  RNG_IRQHandler                    /* RNG */
  .word  FPU_IRQHandler                    /* FPU */
  .word  UART7_IRQHandler                  /* UART7 */
  .word  UART8_IRQHandler                  /* UART8 */
  .word  SPI4_IRQHandler                   /* SPI4 */
  .word  SPI5_IRQHandler                   /* SPI5 */
  .word  SPI6_IRQHandler                   /* SPI6 */
  .word  SAI1_IRQHandler                   /* SAI1 */
  .word  LTDC_IRQHandler                   /* LTDC */
  .word  LTDC_ER_IRQHandler                /* LTDC Error */
  .word  DMA2D_IRQHandler                  /* DMA2D */
  .word  0                                 /* Reserved */
  .word  OCTOSPI1_IRQHandler               /* OCTOSPI1 */
  .word  LPTIM1_IRQHandler                 /* LPTIM1 */
  .word  CEC_IRQHandler                    /* CEC */
  .word  I2C4_EV_IRQHandler                /* I2C4 Event */
  .word  I2C4_ER_IRQHandler                /* I2C4 Error */
  .word  SPDIF_RX_IRQHandler               /* SPDIF RX */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  DFSDM1_FLT0_IRQHandler            /* DFSDM1 Filter0 */
  .word  DFSDM1_FLT1_IRQHandler            /* DFSDM1 Filter1 */
  .word  DFSDM1_FLT2_IRQHandler            /* DFSDM1 Filter2 */
  .word  DFSDM1_FLT3_IRQHandler            /* DFSDM1 Filter3 */
  .word  SDMMC2_IRQHandler                 /* SDMMC2 */
  .word  CAN3_TX_IRQHandler                /* CAN3 TX */
  .word  CAN3_RX0_IRQHandler               /* CAN3 RX0 */
  .word  CAN3_RX1_IRQHandler               /* CAN3 RX1 */
  .word  CAN3_SCE_IRQHandler               /* CAN3 SCE */
  .word  JPEG_IRQHandler                   /* JPEG */
  .word  MDIOS_IRQHandler                  /* MDIOS */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  MDMA_IRQHandler                   /* MDMA */
  .word  0                                 /* Reserved */
  .word  SDMMC3_IRQHandler                 /* SDMMC3 */
  .word  DAM_IRQHandler                    /* DAM */
  .word  0                                 /* Reserved */
  .word  OCTOSPI2_IRQHandler               /* OCTOSPI2 */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  FMAC_IRQHandler                   /* FMAC */
  .word  CORDIC_IRQHandler                 /* CORDIC */
  .word  UART9_IRQHandler                  /* UART9 */
  .word  USART10_IRQHandler                /* USART10 */
  .word  I2C5_EV_IRQHandler                /* I2C5 Event */
  .word  I2C5_ER_IRQHandler                /* I2C5 Error */
  .word  I2C6_EV_IRQHandler                /* I2C6 Event */
  .word  I2C6_ER_IRQHandler                /* I2C6 Error */
  .word  AES_IRQHandler                    /* AES */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  0                                 /* Reserved */
  .word  WAKEUP_PIN_IRQHandler             /* Wakeup Pin */

.section .text.Reset_Handler
.type   Reset_Handler, %function

Reset_Handler:
  ldr   sp, =_estack       /* Set stack pointer */

  /* Copy .data from FLASH to SRAM */
  ldr   r0, =_sdata        /* Destination: SRAM */
  ldr   r1, =_edata        /* End of destination */
  ldr   r2, =_sidata       /* Source: FLASH */
  b     .L_copy_loop
.L_copy_word:
  ldr   r3, [r2], #4       /* Load word from FLASH */
  str   r3, [r0], #4       /* Store word to SRAM */
.L_copy_loop:
  cmp   r0, r1             /* Check if done */
  bcc   .L_copy_word       /* Continue if not done */

  /* Zero fill .bss */
  ldr   r0, =_sbss         /* Start of .bss */
  ldr   r1, =_ebss         /* End of .bss */
  movs  r2, #0             /* Zero value */
  b     .L_bss_loop
.L_bss_word:
  str   r2, [r0], #4       /* Store zero */
.L_bss_loop:
  cmp   r0, r1             /* Check if done */
  bcc   .L_bss_word        /* Continue if not done */

  /* Call SystemInit */
  bl    SystemInit

  /* Call main */
  bl    main

  /* Loop forever if main returns */
.L_loop:
  b     .L_loop

.size Reset_Handler, .-Reset_Handler

/* Default Handler - infinite loop */
.section .text.Default_Handler, "ax", %progbits
.type   Default_Handler, %function
Default_Handler:
  b     Default_Handler
.size Default_Handler, .-Default_Handler

/* Weak aliases for interrupt handlers */
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler
  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler
  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler
  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler
  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler
  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler, Default_Handler
  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler
  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler

  /* External interrupts - all default to Default_Handler */
  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, Default_Handler
  .weak PVD_AVD_IRQHandler
  .thumb_set PVD_AVD_IRQHandler, Default_Handler
  .weak TAMP_IRQHandler
  .thumb_set TAMP_IRQHandler, Default_Handler
  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler, Default_Handler
  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, Default_Handler
  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler, Default_Handler
  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, Default_Handler
  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, Default_Handler
  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, Default_Handler
  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, Default_Handler
  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, Default_Handler
  .weak DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler, Default_Handler
  .weak DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler, Default_Handler
  .weak DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler, Default_Handler
  .weak DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler, Default_Handler
  .weak DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler, Default_Handler
  .weak DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler, Default_Handler
  .weak DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler, Default_Handler
  .weak ADC_IRQHandler
  .thumb_set ADC_IRQHandler, Default_Handler
  .weak EXTI5_9_IRQHandler
  .thumb_set EXTI5_9_IRQHandler, Default_Handler
  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler, Default_Handler
  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler, Default_Handler
  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler
  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, Default_Handler
  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, Default_Handler
  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, Default_Handler
  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, Default_Handler
  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, Default_Handler
  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, Default_Handler
  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, Default_Handler
  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, Default_Handler
  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler, Default_Handler
  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler, Default_Handler
  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler, Default_Handler
  .weak EXTI10_15_IRQHandler
  .thumb_set EXTI10_15_IRQHandler, Default_Handler
  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler, Default_Handler
  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, Default_Handler
  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, Default_Handler
  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler, Default_Handler
  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler, Default_Handler
  .weak TIM6_DAC_IRQHandler
  .thumb_set TIM6_DAC_IRQHandler, Default_Handler
  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler, Default_Handler
  .weak DMA2_Stream0_IRQHandler
  .thumb_set DMA2_Stream0_IRQHandler, Default_Handler
  .weak DMA2_Stream1_IRQHandler
  .thumb_set DMA2_Stream1_IRQHandler, Default_Handler
  .weak DMA2_Stream2_IRQHandler
  .thumb_set DMA2_Stream2_IRQHandler, Default_Handler
  .weak DMA2_Stream3_IRQHandler
  .thumb_set DMA2_Stream3_IRQHandler, Default_Handler
  .weak DMA2_Stream4_IRQHandler
  .thumb_set DMA2_Stream4_IRQHandler, Default_Handler
  .weak ETH_IRQHandler
  .thumb_set ETH_IRQHandler, Default_Handler
  .weak ETH_WKUP_IRQHandler
  .thumb_set ETH_WKUP_IRQHandler, Default_Handler
  .weak DMA2_Stream5_IRQHandler
  .thumb_set DMA2_Stream5_IRQHandler, Default_Handler
  .weak DMA2_Stream6_IRQHandler
  .thumb_set DMA2_Stream6_IRQHandler, Default_Handler
  .weak DMA2_Stream7_IRQHandler
  .thumb_set DMA2_Stream7_IRQHandler, Default_Handler
  .weak USART6_IRQHandler
  .thumb_set USART6_IRQHandler, Default_Handler
  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler, Default_Handler
  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler, Default_Handler

.end