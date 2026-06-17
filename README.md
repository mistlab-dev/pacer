# PACER 飞控 — STM32H743 + FreeRTOS

四旋翼飞控项目，从 RPi Zero 迁移到 STM32H743VI 裸机平台。

## 项目结构

```
pacer/
├── CMakeLists.txt            # 交叉编译配置
├── cmake/
│   ├── arm-toolchain.cmake   # ARM GCC 工具链
│   ├── stm32h743vi_flash.ld  # 链接脚本
│   └── startup_stm32h743xx.s # 启动汇编
├── include/                  # 公共头文件
│   └── config.h              # 全局配置参数
├── src/
│   ├── main.c                # 入口 + 系统时钟
│   ├── hal/                  # 硬件抽象层
│   │   ├── hal_i2c.c
│   │   ├── hal_gpio.c
│   │   ├── hal_tim.c
│   │   ├── usart_printf.c
│   │   └── system_stm32h7xx.c
│   ├── sensor/               # IMU 驱动
│   │   ├── imu.c
│   │   └── impl/imu_icm20948.c
│   ├── filter/               # 姿态滤波
│   │   └── filter.c
│   ├── ctrl/                 # 控制算法
│   │   ├── pid.c
│   │   ├── attitude.c
│   │   └── quad_mixer.c
│   ├── motor/                # ESC 电机驱动
│   │   └── motor.c
│   ├── remote/               # UART 遥控
│   │   └── remote.c
│   └── app/                  # FreeRTOS 应用层
│       └── app.c
├── cmsis/                    # CMSIS 核心头文件
│   └── Include/core_cm7.h
├── stm32hal/                 # STM32 HAL 库 (需补充)
│   ├── cmsis_device/Include/ # 设备头文件 (需补充)
│   └── hal/                  # HAL 源码 (需补充)
├── freertos/                 # FreeRTOS (需补充)
│   └── FreeRTOSConfig.h
├── tools/
│   └── pacer-remote/         # Go 遥控上位机
├── scripts/
│   ├── build.bat             # Windows 编译
│   ├── flash.bat             # Windows 烧录
│   ├── build.sh              # Linux 编译
│   ├── flash.sh              # Linux 烧录
│   └── README.md             # 环境配置说明
└── docs/
    └── SETUP.md              # ← 本文件，新手必读
```

## 快速开始

1. 先读本文件 `docs/SETUP.md`，完成 CubeMX 依赖生成
2. 编译：`scripts/build.bat`（Windows）或 `./scripts/build.sh`（Linux）
3. 烧录：`scripts/flash.bat`（Windows）或 `./scripts/flash.sh`（Linux）

## 硬件配置

- MCU: STM32H743VI @ 480MHz
- IMU: ICM20948 (I2C1: PB6/PB7)
- ESC: 4× 无刷电调 (TIM1: PA8/PE11/PE13/PE14)
- 遥控: USART3 (PD8/PD9)
- 调试: USART2 (PA2/PA3)
