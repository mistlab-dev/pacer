# Pacer v3.0 — 四旋翼无人机飞控

树莓派 Zero 2W 上的四旋翼飞控，纯 C 实现。

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| 主控 | Raspberry Pi Zero 2W | — |
| IMU | ICM20948 (9轴) | I2C-1, addr 0x68 |
| PWM | PCA9685 (16路) | I2C-1, addr 0x40 |
| 电机 | 4× 无刷电机 + ESC | PCA9685 CH0~CH3 |

## 电机布局 ("X" 型)

```
     前
    FL  FR      FL=前左(CW)   FR=前右(CCW)
     \/
     /\
    RL  RR      RL=后左(CCW)  RR=后右(CW)
     后
```

## 架构

```
hal/        硬件抽象 (GPIO, I2C, PCA9685)
sensor/     IMU 驱动 (ICM20948)
filter/     姿态估计 (Madgwick / 互补滤波)
ctrl/       控制算法
  pid         通用 PID
  attitude    姿态控制器 (串级 PID)
  quad_mixer  四旋翼混控器
motor/      电机驱动 (ESC)
remote/     遥控输入 (键盘/UDP)
app/        应用层 (主循环, 模式管理)
```

## 控制流程

```
遥控输入 (throttle/roll/pitch/yaw)
    │
    ├─ throttle ──────────────────→ 混控器
    │
    ├─ roll/pitch → 姿态控制器 → 混控器
    │                  ├─ 角度环 (自稳模式)
    │                  └─ 角速度环 (内环)
    │
    └─ yaw_rate → 角速度环 → 混控器
                     │
               混控器 (X型公式)
               FL = thr - pit + rol - yaw
               FR = thr - pit - rol + yaw
               RL = thr + pit + rol + yaw
               RR = thr + pit - rol - yaw
                     │
                 4× ESC → 电机
```

## 编译

```bash
mkdir build && cd build
cmake ..
make
```

目标机器需要安装 pigpio: `sudo apt install pigpio`

## 运行

```bash
# 正常飞行
sudo ./pacer

# 校准 IMU 陀螺仪 (首次使用)
sudo ./pacer --calibrate

# 校准 ESC 油门行程 (新电调)
sudo ./pacer --esc-cal

# 调试模式 (打印姿态数据, 不驱动电机)
sudo ./pacer --debug
```

## 遥控

### 键盘 (默认, 调试用)

| 按键 | 功能 |
|------|------|
| R | 解锁/上锁 |
| W/S | pitch 前/后 |
| A/D | roll 左/右 |
| Q/E | yaw 左旋/右旋 |
| 1-9 | 油门 10%~90% |
| 0 | 油门归零 |
| 空格 | 全部归零 |
| X | 紧急停止 |

### UDP

16 字节, 4× float (小端):

```
[throttle] [roll] [pitch] [yaw]
  0~1      -1~+1   -1~+1   -1~+1
```

端口: 8888 (config.h 中修改)

## 调参

核心 PID 参数在 `include/app/config.h`:

- `CFG_QUAD_*_RATE_KP/KI/KD` — 角速度环 (最重要)
- `CFG_QUAD_*_ANGLE_KP/KI/KD` — 角度环 (自稳模式)
- `CFG_QUAD_MAX_*_RATE` — 角速度限制
- `CFG_MIXER_HOVER` — 悬停油门

## 安全

- 倾斜超过 60° 自动急停
- 遥控信号丢失 0.5 秒自动归零
- 上锁状态下电机无输出
- Ctrl+C 信号安全退出

## License

MIT
