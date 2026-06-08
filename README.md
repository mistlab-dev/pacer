# Pacer — 二轮平衡车

> 树莓派 Zero 2W + ICM20948 九轴 IMU，纯 C 实现，模块化设计。

## 架构

```
┌─────────────────────────────────────────────────┐
│                    main.c                        │
│               (参数解析, 入口)                    │
├─────────────────────────────────────────────────┤
│                     app/                         │
│          系统初始化 · 主循环 · 模式管理             │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│   │ sensor/  │  │ filter/  │  │  ctrl/   │      │
│   │   imu    │→ │ attitude │→ │ balance  │      │
│   │ (ICM20948)│  │(Madgwick)│  │  (PID)   │      │
│   └──────────┘  └──────────┘  └──────────┘      │
│                        ↓              ↓          │
│               ┌──────────┐  ┌──────────┐        │
│               │  hal/    │  │  motor/  │        │
│               │ (i2c,gpio)│  │  (PWM)  │        │
│               └──────────┘  └──────────┘        │
└─────────────────────────────────────────────────┘
```

### 模块说明

| 模块 | 职责 | 可替换性 |
|------|------|----------|
| **hal/** | 硬件抽象 (I2C, GPIO, PWM) | 移植到其他平台只改这里 |
| **sensor/** | IMU 接口 + ICM20948 实现 | 换传感器只加一个 impl 文件 |
| **filter/** | 姿态滤波 (Madgwick / 互补) | config.h 一行切换 |
| **ctrl/** | PID 控制器 + 平衡策略 | 参数在 config.h 调 |
| **motor/** | 双向 PWM 电机驱动 | inverted 标志适配安装方向 |
| **app/** | 配置 + 初始化 + 主循环 + 模式 | 业务逻辑 |

### 关键设计

- **策略模式**: IMU 驱动通过函数指针注册, 上层不绑定具体芯片
- **HAL 抽象**: 所有硬件操作通过 hal 层, 方便移植
- **单文件单模块**: 每个模块一对 .h/.c, 职责清晰
- **零动态内存**: 全部静态分配, 适合嵌入式

## 硬件接线

```
树莓派 Zero 2W        ICM20948
─────────────        ────────
GPIO2  (SDA)  ──→    SDA
GPIO3  (SCL)  ──→    SCL
3.3V          ──→    VCC
GND           ──→    GND

树莓派 Zero 2W        电机驱动板 (TB6612 等)
─────────────        ────────────────────
GPIO12 (PWM)  ──→    左电机 AIN1
GPIO13 (PWM)  ──→    左电机 AIN2
GPIO18 (PWM)  ──→    右电机 BIN1
GPIO19 (PWM)  ──→    右电机 BIN2
```

## 编译 & 运行

```bash
# 依赖 (树莓派)
sudo apt install pigpio libpigpio-dev cmake build-essential

# 编译
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 校准 (首次)
sudo ./pacer --calibrate

# 调试 (看姿态数据)
sudo ./pacer --debug

# 开跑!
sudo ./pacer
```

## 调参

编辑 `include/app/config.h`:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CFG_BAL_ANGLE_KP` | 25.0 | 角度环比例 — **第一个调的参数** |
| `CFG_BAL_ANGLE_KD` | 0.8  | 角度环微分 — 消振荡 |
| `CFG_BAL_ANGLE_OFFSET` | 0.0 | 机械零点 (车总往一边倒就改这个) |
| `CFG_CONTROL_HZ` | 200 | 控制频率 |

### 调参口诀

1. Kp 从小到大, 直到车能短暂站立
2. 加 Kd 消除抖动
3. 最后调速度环, 让车保持位置

## 项目结构

```
pacer/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── hal/            # 硬件抽象层
│   ├── sensor/         # IMU 接口 + 驱动注册
│   ├── filter/         # 姿态滤波器接口
│   ├── ctrl/           # PID + 平衡控制
│   ├── motor/          # 电机驱动接口
│   └── app/            # 配置 + 应用层
├── src/
│   ├── hal/            # pigpio 实现
│   ├── sensor/         # IMU 接口实现
│   ├── sensor/impl/    # ICM20948 实现
│   ├── filter/         # Madgwick + 互补滤波
│   ├── ctrl/           # PID + 三环串级
│   ├── motor/          # PWM 电机
│   ├── app/            # 主循环
│   └── main.c          # 入口
└── scripts/
    └── deploy.sh
```

## License

MIT
