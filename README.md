# Pacer — 四轮差速驱动车

> 树莓派 Zero 2W + PCA9685 + 无刷电调，纯 C 实现，模块化设计。
> 目标速度 15 km/h，后续计划加装升降机构实现爬楼梯。

## 架构

```
┌─────────────────────────────────────────────────────────┐
│                       main.c                             │
│                  (参数解析, 入口)                          │
├─────────────────────────────────────────────────────────┤
│                      app/                                │
│           系统初始化 · 主循环 · 模式管理                    │
│                                                          │
│  ┌────────────┐                                          │
│  │  remote/   │──── 遥控输入 (键盘/UDP)                   │
│  └────────────┘                                          │
│         ↓                                                │
│  ┌────────────┐                                          │
│  │ ctrl/      │                                          │
│  │ diff_drive │──── 差速混合: left=thr-steer, right=thr+steer │
│  └────────────┘                                          │
│         ↓                                                │
│  ┌────────────┐    ┌────────────┐    ┌────────────┐      │
│  │  motor/    │    │  hal/      │    │  sensor/   │      │
│  │  (DC/ESC)  │←── │ PCA9685    │    │  IMU       │      │
│  └────────────┘    │ I2C/GPIO   │    └────────────┘      │
│                    └────────────┘                         │
└─────────────────────────────────────────────────────────┘
```

### 模块说明

| 模块 | 职责 | 可替换性 |
|------|------|----------|
| **hal/** | 硬件抽象 (I2C, GPIO, PCA9685) | 移植到其他平台只改这里 |
| **motor/** | 统一电机接口 (DC 双通道 / ESC 舵机信号) | 切换电机类型只改 config |
| **ctrl/** | 差速驱动控制器 + PID | 参数在 config.h 调 |
| **remote/** | 遥控输入 (键盘/UDP) | 扩展其他遥控方式 |
| **sensor/** | IMU 接口 (ICM20948) | 换传感器只加一个 impl 文件 |
| **app/** | 配置 + 初始化 + 主循环 | 业务逻辑 |

### 电机模式

| 模式 | 配置 | PCA9685 通道 | 说明 |
|------|------|-------------|------|
| DC 有刷 | `CFG_MOTOR_MODE=0` | 每电机 2 通道 (IN1/IN2) | TT马达、380 等 |
| **ESC 无刷** | `CFG_MOTOR_MODE=1` | 每电机 1 通道 (信号线) | **当前方案** |

## 硬件清单

| 组件 | 数量 | 说明 |
|------|------|------|
| 树莓派 Zero 2W | 1 | 主控 |
| ICM20948 九轴 IMU | 1 | 姿态传感器 |
| PCA9685 PWM 扩展板 | 1 | I2C, 16路 PWM |
| 无刷电机 + 电调 (ESC) | 4 | 双向, 12V |
| 3S 锂电池 (11.1V) | 1 | 动力电源 |
| 80mm 橡胶轮 | 4 | 铝合金轮毂 |
| 铝合金底盘 | 1 | 四轮差速 |

## 接线

```
树莓派 Zero 2W        PCA9685          电调 (ESC) ×4
─────────────        ────────         ──────────────
GPIO2 (SDA)  ──→    SDA
GPIO3 (SCL)  ──→    SCL
3.3V         ──→    VCC
GND          ──→    GND
                    CH0    ──→    前左 ESC 信号线
                    CH1    ──→    前右 ESC 信号线
                    CH2    ──→    后左 ESC 信号线
                    CH3    ──→    后右 ESC 信号线

3S 锂电池 (11.1V)
    ├──→ 4个电调电源线 (红/黑)
    └──→ 树莓派 5V 降压模块 (可选)

树莓派 Zero 2W        ICM20948
─────────────        ────────
GPIO2  (SDA)  ──→    SDA     (共用 I2C)
GPIO3  (SCL)  ──→    SCL
3.3V          ──→    VCC
GND           ──→    GND
```

## 编译 & 运行

```bash
# 依赖 (树莓派)
sudo apt install pigpio libpigpio-dev cmake build-essential

# 编译
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 校准 IMU
sudo ./pacer --calibrate

# 调试 (看姿态数据)
sudo ./pacer --debug

# 开跑! (键盘 WASD 遥控)
sudo ./pacer
```

## 遥控

| 按键 | 动作 |
|------|------|
| W | 前进 (50% 油门) |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| Q | 前进+左转 |
| E | 前进+右转 |
| 空格 | 停止 |
| X | 退出程序 |

## ESC 安全机制

1. **初始化**: 启动时自动发送中位信号 (1500μs)
2. **解锁 (Arm)**: 等待 3 秒，让电调识别中位
3. **未解锁保护**: 未完成 arm 流程前，`motor_set()` 不输出
4. **信号丢失**: 程序退出/崩溃时 PCA9685 保持最后状态（需配合看门狗）

## 调参

编辑 `include/app/config.h`:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CFG_ESC_FL_CH` | 0 | 前左 ESC 通道 |
| `CFG_ESC_FR_CH` | 1 | 前右 ESC 通道 |
| `CFG_ESC_RL_CH` | 2 | 后左 ESC 通道 |
| `CFG_ESC_RR_CH` | 3 | 后右 ESC 通道 |
| `CFG_DIFF_SPEED_KP` | 1.0 | 速度环比例 |
| `CFG_DIFF_CONTROL_HZ` | 100 | 控制频率 |

## 项目结构

```
pacer/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── hal/              # 硬件抽象层 (GPIO, I2C, PCA9685)
│   ├── motor/            # 统一电机接口 (DC / ESC)
│   ├── ctrl/             # PID + 差速驱动
│   ├── sensor/           # IMU 接口
│   ├── filter/           # 姿态滤波
│   └── app/              # 配置 + 应用层
├── src/
│   ├── hal/              # pigpio + PCA9685 实现
│   ├── motor/            # DC / ESC 电机驱动
│   ├── ctrl/             # PID + diff_drive
│   ├── sensor/           # IMU + ICM20948
│   ├── filter/           # Madgwick + 互补滤波
│   ├── app/              # 主循环
│   └── main.c            # 入口
└── tests/                # 单元测试 (mock HAL)
    ├── mocks/            # GPIO + I2C mock
    ├── test_pid.c
    ├── test_motor.c
    ├── test_pca9685.c
    └── test_diff_drive.c
```

## 测试

```bash
cd tests && mkdir build && cd build
cmake .. && make
make check   # 运行全部测试
```

## 升降机构 (规划中)

PCA9685 CH8~CH9 预留给升降舵机/电机：
- 每个 PCA9685 还有 12 个空闲通道
- config.h 已预留 `CFG_LIFT_*` 配置
- 计划：独立控制每个轮子的升降高度，实现爬楼梯

## License

MIT
