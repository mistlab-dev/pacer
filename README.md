# Pacer v3.0 — 四旋翼无人机飞控

树莓派 Zero / Zero 2W 四旋翼飞控，纯 C (C11) 实现。支持两种板子：

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| MCU | Raspberry Pi Zero / Zero 2W | — |
| IMU | ICM20948 (9轴) | I2C bus 1, addr 0x68 |
| 电机驱动 | PCA9685 (16路PWM) | I2C bus 1, addr 0x40 |
| 电机 | 4× 无刷电调 (ESC) | PCA9685 CH0~CH3 |

### 机架布局 ("X" 型)

```
     前
  FL(CW)  FR(CCW)
      ╲  ╱
       ╲╱
       ╱╲
      ╱  ╲
  RL(CCW) RR(CW)
     后
```

- FL=前左(顺时针), FR=前右(逆时针)
- RL=后左(逆时针), RR=后右(顺时针)
- ESC 通道: FL=CH0, FR=CH1, RL=CH2, RR=CH3

## 功能

- **姿态控制**: 串级 PID (角度环 + 角速度环)
- **两种模式**: 角速度模式 (Acro) / 自稳模式 (Self-level)
- **混控器**: "X" 型四旋翼混控，自动限幅
- **遥控**: 键盘 (调试) 或 UDP 16 字节二进制协议
- **安全**: 解锁/上锁状态机, 急停, 倾斜保护 (60°)
- **控制频率**: Zero 2W 400 Hz / Zero 200 Hz
- **IMU 采样**: 与控制频率同步, 加速度 ±8g, 陀螺仪 ±1000°/s

## 构建

### Raspberry Pi Zero 2W (四核 ARMv8, 400Hz)

```bash
mkdir build && cd build
cmake -DTARGET_BOARD=rpi_zero2w .. && make
```

### Raspberry Pi Zero (单核 ARMv6, 200Hz)

```bash
mkdir build && cd build
cmake -DTARGET_BOARD=rpi_zero .. && make
```

### 默认 (Zero 2W)

```bash
mkdir build && cd build
cmake .. && make
```

### 运行

```bash
# 正常飞行
sudo ./pacer

# 校准 IMU 陀螺仪零偏
sudo ./pacer --calibrate

# 校准 ESC 油门行程
sudo ./pacer --esc-cal

# 调试模式 (打印姿态, 不驱动电机)
sudo ./pacer --debug
```

### 模拟测试 (不需要硬件)

```bash
gcc -Wall -I include -o build/sim_quad tests/sim_quad.c \
    src/ctrl/pid.c src/ctrl/attitude.c src/ctrl/quad_mixer.c -lm
./build/sim_quad
```

### 单元测试

```bash
cd tests
mkdir build && cd build
cmake .. && make
make check
```

## 遥控协议

### UDP (端口 8888)

16 字节, 小端:
```
float throttle  (0.0 ~ 1.0)
float roll      (-1.0 ~ +1.0, 正=右倾)
float pitch     (-1.0 ~ +1.0, 正=前倾)
float yaw       (-1.0 ~ +1.0, 正=顺时针)
```

### 键盘 (调试)

| 键 | 功能 |
|----|------|
| W/S | Pitch 前/后 |
| A/D | Roll 左/右 |
| Q/E | Yaw 左/右 |
| 1-9 | 油门 10%~90% |
| R | 解锁/上锁 |
| X | 急停 |

## 代码结构

```
include/
  app/config.h        — 全局配置 (频率、PID 参数、安全阈值)
  app/app.h           — 主循环接口
  ctrl/pid.h          — 通用 PID 控制器
  ctrl/attitude.h     — 姿态控制器 (串级)
  ctrl/quad_mixer.h   — 四旋翼混控器
  filter/filter.h     — 互补滤波 / 低通滤波
  sensor/imu.h        — IMU 接口
  motor/motor.h       — 电机驱动 (DC/ESC)
  remote/remote.h     — 遥控输入
  hal/                — 硬件抽象层 (I2C, GPIO, PCA9685)
src/
  (对应实现)
tests/
  sim_quad.c          — 控制逻辑全面模拟测试
  test_pid.c          — PID 单元测试
  test_motor.c        — 电机驱动测试
  test_pca9685.c      — PCA9685 测试
```

## 混控公式

```
FL = T - P + R - Y
FR = T - P - R + Y
RL = T + P + R + Y
RR = T + P - R - Y

T = throttle (油门)
P = pitch 校正量
R = roll 校正量
Y = yaw 校正量
```

## PID 参数 (初始值, 需实际调试)

| 参数 | Roll/Pitch 角速度 | Yaw 角速度 | Roll/Pitch 角度 |
|------|-------------------|-----------|----------------|
| Kp | 0.40 | 0.30 | 4.0 |
| Ki | 0.05 | 0.02 | 0.1 |
| Kd | 0.01 | 0.00 | 0.0 |

## 版本历史

- **v1.0** — 两轮平衡小车
- **v2.0** — 四轮差速驱动车 (ESC 电机)
- **v3.0** — 四旋翼无人机 (当前)
