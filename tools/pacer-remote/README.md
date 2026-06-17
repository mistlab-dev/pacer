# PACER 遥控上位机

Go 串口遥控工具，发送控制命令到 STM32H743 飞控。

## 协议

18 字节帧，小端序：

```
[0xAA] [0x55] [throttle:4] [roll:4] [pitch:4] [yaw:4]
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| throttle | float32 | 0~1 | 油门，0=停，1=满 |
| roll | float32 | -1~+1 | 滚转，左负右正 |
| pitch | float32 | -1~+1 | 俯仰，前负后正 |
| yaw | float32 | -1~+1 | 偏航，左负右正 |

## 编译

```bash
cd tools/pacer-remote
go build -o pacer-remote .
```

Windows 下生成 `pacer-remote.exe`。

## 使用

```bash
# 基本用法
pacer-remote -port COM3 -baud 115200

# 设置初始值
pacer-remote -port COM3 -t 0.5 -r 0.1 -p -0.1 -y 0

# Demo 模式 (自动摇摆)
pacer-remote -port COM3 -demo

# Linux
pacer-remote -port /dev/ttyUSB0 -t 0.3
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-port` | 必填 | 串口设备名 |
| `-baud` | 115200 | 波特率 |
| `-freq` | 50 | 发送频率 Hz |
| `-t` | 0 | 初始油门 |
| `-r` | 0 | 初始滚转 |
| `-p` | 0 | 初始俯仰 |
| `-y` | 0 | 初始偏航 |
| `-demo` | false | 演示模式 |

## Demo 模式

自动执行周期性摇摆动作：

- 油门固定 0.5
- 滚转 ±0.3 周期摆动
- 俯仰 ±0.2 周期摆动

用于测试飞控响应，**请确保电机已正确安装且有安全防护**。

## 交互式控制

当前版本只支持 Ctrl+C 退出。完整键盘控制需要额外依赖：

```bash
go get golang.org/x/term
```

后续版本可扩展为 WASD 键盘实时控制。

## 注意

- 启动前确保飞控已上电、串口已连接
- 退出时会自动发送归零命令
- Demo 模式下 Ctrl+C 会先归零再退出
- **安全第一**: 首次测试建议拆掉电机螺旋桨