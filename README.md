# Pacer — 二轮平衡车

树莓派 Zero 2W + ICM20948 九轴 IMU，C 语言实现。

## 功能

- **姿态解算** — Madgwick 滤波器 / 互补滤波
- **三环串级 PID** — 角度环 + 速度环 + 转向环
- **ICM20948 驱动** — I2C 读取加速度计 + 陀螺仪 + 磁力计
- **电机 PWM 驱动** — pigpio 硬件 PWM
- **自动校准** — 陀螺仪零偏校准
- **紧急停机** — 倾角过大自动保护

## 硬件接线

```
树莓派 Zero 2W        ICM20948
─────────────        ────────
GPIO2 (SDA)    ───→  SDA
GPIO3 (SCL)    ───→  SCL
3.3V           ───→  VCC
GND            ───→  GND

树莓派 Zero 2W        电机驱动 (TB6612/L298N)
─────────────        ────────────────────────
GPIO12 (PWM0)  ───→  左电机 AIN1
GPIO13 (PWM1)  ───→  左电机 AIN2
GPIO18 (PWM0)  ───→  右电机 BIN1
GPIO19 (PWM1)  ───→  右电机 BIN2
```

## 编译 & 运行

```bash
# 安装依赖 (树莓派)
sudo apt install pigpio libpigpio-dev cmake build-essential

# 编译
mkdir build && cd build
cmake ..
make -j$(nproc)

# 校准 IMU (首次使用)
sudo ./pacer --calibrate

# 调试模式 (查看姿态数据)
sudo ./pacer --debug

# 正式运行
sudo ./pacer
```

## 参数调优

编辑 `include/config.h`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| BALANCE_KP | 25.0 | 平衡环比例增益 |
| BALANCE_KI | 0.0 | 平衡环积分增益 |
| BALANCE_KD | 0.8 | 平衡环微分增益 |
| BALANCE_ANGLE_OFFSET | 0.0 | 机械零点偏移 |
| CONTROL_RATE_HZ | 200 | 控制循环频率 |
| ANGLE_MAX | 30.0 | 允许最大倾角 |

### 调参步骤

1. **先调平衡环** — Kp 从小到大，直到车能短暂站立
2. **加微分** — Kd 消除振荡
3. **最后调速度环** — 让车能保持位置
4. **机械零点** — 如果车总是往一边倒，调整 ANGLE_OFFSET

## 项目结构

```
pacer/
├── CMakeLists.txt
├── include/
│   ├── config.h              # 全局配置
│   ├── sensors/
│   │   ├── imu.h             # ICM20948 驱动
│   │   └── attitude.h        # 姿态解算
│   ├── control/
│   │   ├── pid.h             # PID 控制器
│   │   └── balance.h         # 平衡控制
│   ├── drivers/
│   │   └── motor.h           # 电机驱动
│   └── utils/
│       └── logger.h          # 日志
├── src/
│   ├── main.c                # 主程序
│   ├── sensors/
│   │   ├── imu.c
│   │   └── attitude.c
│   ├── control/
│   │   ├── pid.c
│   │   └── balance.c
│   ├── drivers/
│   │   └── motor.c
│   └── utils/
│       └── logger.c
└── scripts/
    └── deploy.sh             # 部署脚本
```

## License

MIT
