/**
 * @file config.h
 * @brief 全局运行时配置 — 四旋翼无人机 (STM32H743)
 *
 * 编译时常量。运行时参数通过接口函数修改，方便在线调参。
 */

#ifndef PACER_CONFIG_H
#define PACER_CONFIG_H

/* ==================== 目标平台 ==================== */

#define TARGET_STM32H743        1
#define TARGET_MCU              "STM32H743"
#define SYS_CLOCK_HZ            256000000U   /* HSI+PLL 256MHz, APB 64MHz */

/* ==================== IMU ==================== */

#define CFG_IMU_I2C_ADDR          0x68    /* ICM20948 AD0=0 */
#define CFG_IMU_SAMPLE_HZ         400
#define CFG_IMU_ACCEL_RANGE_G     8       /* ±8g */
#define CFG_IMU_GYRO_RANGE_DPS    1000    /* ±1000°/s */

/* ==================== 控制频率 ==================== */

#define CFG_CONTROL_HZ            400
#define CFG_CONTROL_DT            (1.0f / CFG_CONTROL_HZ)
#define CFG_CONTROL_INTERVAL_US   (1000000 / CFG_CONTROL_HZ)

/* ==================== 电机 PWM (硬件 Timer) ==================== */

/*
 * H743 Timer PWM 配置:
 *   ESC 输出 50Hz, 1000~2000μs 脉宽
 *   Timer 时钟 = 128MHz (PCLK2 64MHz × 2)
 *   预分频 = 128-1 → 计数频率 1MHz (1μs/tick)
 *   自动重载 = 20000-1 → 周期 20ms = 50Hz
 *
 *   脉宽映射: 1000~2000 ticks = 1000~2000μs
 */
#define CFG_ESC_PWM_FREQ_HZ       50
#define CFG_ESC_PWM_PRESCALER     127     /* 128MHz / 128 = 1MHz */
#define CFG_ESC_PWM_PERIOD        20000   /* 20ms = 50Hz */
#define CFG_ESC_PULSE_MIN         1000    /* 1ms = 最低油门/停止 */
#define CFG_ESC_PULSE_MAX         2000    /* 2ms = 最大油门 */
#define CFG_ESC_PULSE_ARM         1000    /* 解锁脉冲 */
#define CFG_ESC_PULSE_NEUTRAL     1500    /* 中位 */

/*
 * Timer / 通道分配:
 *   TIM1: CH1=FL(PA8),  CH2=FR(PE11), CH3=RL(PE13), CH4=RR(PE14)
 *   (H743 高级定时器, 互补输出可配, 这里用主通道)
 */
#define CFG_ESC_FL_TIMER          TIM1
#define CFG_ESC_FL_CHANNEL        TIM_CHANNEL_1
#define CFG_ESC_FR_TIMER          TIM1
#define CFG_ESC_FR_CHANNEL        TIM_CHANNEL_2
#define CFG_ESC_RL_TIMER          TIM1
#define CFG_ESC_RL_CHANNEL        TIM_CHANNEL_3
#define CFG_ESC_RR_TIMER          TIM1
#define CFG_ESC_RR_CHANNEL        TIM_CHANNEL_4

/* ==================== 姿态控制 PID ==================== */

/* 角速度环 (内环 — 核心) */
#define CFG_QUAD_ROLL_RATE_KP      0.40f
#define CFG_QUAD_ROLL_RATE_KI      0.05f
#define CFG_QUAD_ROLL_RATE_KD      0.01f

#define CFG_QUAD_PITCH_RATE_KP     0.40f
#define CFG_QUAD_PITCH_RATE_KI     0.05f
#define CFG_QUAD_PITCH_RATE_KD     0.01f

#define CFG_QUAD_YAW_RATE_KP      0.30f
#define CFG_QUAD_YAW_RATE_KI      0.02f
#define CFG_QUAD_YAW_RATE_KD      0.00f

/* 角度环 (外环 — 自稳模式) */
#define CFG_QUAD_ROLL_ANGLE_KP     4.0f
#define CFG_QUAD_ROLL_ANGLE_KI     0.0f
#define CFG_QUAD_ROLL_ANGLE_KD     0.0f

#define CFG_QUAD_PITCH_ANGLE_KP    4.0f
#define CFG_QUAD_PITCH_ANGLE_KI    0.0f
#define CFG_QUAD_PITCH_ANGLE_KD    0.0f

/* ==================== 运动限制 ==================== */

#define CFG_QUAD_MAX_ANGLE         30     /* 自稳模式最大倾斜角 (度) */
#define CFG_QUAD_MAX_ROLL_RATE     360    /* 最大 roll 角速度 (度/s) */
#define CFG_QUAD_MAX_PITCH_RATE    360    /* 最大 pitch 角速度 (度/s) */
#define CFG_QUAD_MAX_YAW_RATE      200    /* 最大 yaw 角速度 (度/s) */
#define CFG_QUAD_MAX_TILT_EMERGENCY 60    /* 超过此角度急停 (度) */

/* ==================== 混控 ==================== */

#define CFG_MIXER_THROTTLE_MIN     0.05f  /* 最低油门 (保持转动) */
#define CFG_MIXER_THROTTLE_MAX     1.0f
#define CFG_MIXER_HOVER            0.50f  /* 悬停油门估计 */

/* ==================== 遥控 (UART) ==================== */

/*
 * USART3 (PD8 TX / PD9 RX) 用于遥控
 * 协议: 19 字节帧 (v2 — 含 XOR 校验)
 *   [0]     0xAA    帧头 1
 *   [1]     0x55    帧头 2
 *   [2..5]  throttle (float, 0~1, 小端)
 *   [6..9]  roll     (float, -1~+1, 小端)
 *   [10..13] pitch    (float, -1~+1, 小端)
 *   [14..17] yaw      (float, -1~+1, 小端)
 *   [18]    XOR      前 18 字节异或校验
 */
#define CFG_REMOTE_UART            USART3
#define CFG_REMOTE_UART_BAUD       115200
#define CFG_REMOTE_TIMEOUT_SEC     0.5f

/* ==================== 调试串口 (CLI / printf) ==================== */

/*
 * 鹿小班等板载 CH347「UART0」→ USART1 (PA9/PA10)。
 * 外接 USB-TTL 时改为 PACER_DEBUG_USART2 (PA2/PA3)。
 *
 * 波特率：固件保持 115200；HSI 时钟偏差使线速约 105600，PC 串口须选 105600。
 * 勿把固件改成 105600（会叠偏差，线速约 96800）。有 HSE 后两端可统一 115200。
 */
#define PACER_DEBUG_USART1       1
#define PACER_DEBUG_USART2       2
#define CFG_DEBUG_UART_PORT      PACER_DEBUG_USART1
#define CFG_DEBUG_UART_BAUD      115200

/* ==================== 功能开关 ==================== */

#define CFG_USE_MADGWICK           1      /* 1=Madgwick, 0=互补滤波 */
#define CFG_ENABLE_CONSOLE_LOG     1      /* UART 调试打印 */
#define CFG_UART_PLAIN_DEBUG       1      /* 1=联调模式；IMU+遥控验证通过后改 0 启飞控 */
#define CFG_IMU_DEBUG              1      /* PLAIN_DEBUG 时周期打印 IMU/姿态 */
#define CFG_IMU_DEBUG_HZ           10     /* IMU 调试输出频率 (Hz) */
#define CFG_REMOTE_DEBUG           1      /* PLAIN_DEBUG 时监听 USART3 并打印遥控状态 */
#define CFG_REMOTE_DEBUG_HZ        10     /* 遥控调试输出频率 (Hz) */

/* 与 CFG_DEBUG_UART_PORT / CFG_DEBUG_UART_BAUD 相同，保留兼容旧引用 */
#define CFG_DEBUG_UART             USART1

/* ==================== 飞行阶段 ==================== */

#define CFG_TAKEOFF_THROTTLE       0.55f  /* 起飞油门 (略高于悬停) */
#define CFG_HOVER_THROTTLE         0.50f  /* 悬停油门 */
#define CFG_TAKEOFF_RAMP_RATE      0.3f   /* 起飞油门爬升率 (/s) */
#define CFG_LANDING_DESCENT_RATE   0.15f  /* 降落油门下降率 (/s) */
#define CFG_LANDING_THROTTLE_MIN   0.05f  /* 降落触地油门阈值 */
#define CFG_HOVER_MAX_DRIFT_DEG    15.0f  /* 悬停最大漂移角度 */
#define CFG_PREFLIGHT_LEVEL_DEG    10.0f  /* 自检水平阈值 */

/* ==================== 安全 ==================== */

#define CFG_ARM_STICK_TIMEOUT_SEC  3.0f   /* 摇杆解锁窗口 */
#define CFG_NO_SIGNAL_TIMEOUT_SEC  0.5f   /* 无信号超时自动降落 */

/* ==================== 看门狗 ==================== */

#define CFG_WATCHDOG_TIMEOUT_MS    500     /* IWDG 超时 500ms */

/* ==================== 电池 ==================== */

/* 3S LiPo 电压阈值 */
#define CFG_BATTERY_WARNING_V      10.5f   /* 低压警告 (3.5V/cell) */
#define CFG_BATTERY_CRITICAL_V     9.6f    /* 严重低压 (3.2V/cell), 必须降落 */
#define CFG_BATTERY_SAMPLE_HZ      10      /* 电池采样频率 */

/* ==================== LED ==================== */

#define CFG_LED_PIN                65      /* PE1 = 4*16+1 */

/* ==================== FreeRTOS ==================== */

#define CFG_TASK_CTRL_STACK        1024    /* 控制任务栈 (字) */
#define CFG_TASK_CTRL_PRIORITY     5       /* 控制任务优先级 (高) */
#define CFG_TASK_TELEM_STACK       1536    /* 遥测任务栈 (printf 浮点需要更大栈) */
#define CFG_TASK_TELEM_PRIORITY    2       /* 遥测任务优先级 (低) */
#define CFG_TASK_RX_STACK          512     /* 遥控接收任务栈 */
#define CFG_TASK_RX_PRIORITY       4       /* 遥控接收任务优先级 */

#endif /* PACER_CONFIG_H */
