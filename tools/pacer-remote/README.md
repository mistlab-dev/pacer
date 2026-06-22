# PACER 遥控上位机

Go 串口遥控工具，发送 19 字节帧到 STM32 USART3 或 **地面 ESP32**（ESP-NOW 桥）。

## 协议

19 字节帧，小端序：

```
[0xAA] [0x55] [throttle:4] [roll:4] [pitch:4] [yaw:4] [xor:1]
```

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| throttle | float32 | 0~1 | 油门 |
| roll | float32 | -1~+1 | 滚转，左负右正 |
| pitch | float32 | -1~+1 | 俯仰，前负后正 |
| yaw | float32 | -1~+1 | 偏航，左负右正 |
| xor | uint8 | — | 前 18 字节异或 |

## 编译

```bash
cd tools/pacer-remote
go build -o pacer-remote .
```

Windows 生成 `pacer-remote.exe`。

无 Go 环境可用 Python 替代：`python scripts/remote_send.py --port COMx --demo`

## 使用

```bash
# ESP-NOW 地面端 USB 口
pacer-remote -port COM6 -demo

# 交互式（默认开启）
pacer-remote -port COM6
# 输入: t+ t- r+ p+ y+ z(归零) s(状态) q(退出)

# 固定值、关闭交互
pacer-remote -port COM6 -t 0 -r 0.2 -i=false

# 直连飞控 USART3（少见，一般经 ESP）
pacer-remote -port COM5 -baud 115200
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-port` | 必填 | 串口（地面 ESP 的 COM） |
| `-baud` | 115200 | 波特率 |
| `-freq` | 50 | 发送频率 Hz |
| `-t/-r/-p/-y` | 0 | 初始通道值 |
| `-demo` | false | 演示波形 |
| `-i` | true | 交互式 stdin 命令 |

## 安全

- 首次测试拆桨
- 退出时自动发归零帧
- Demo 模式油门为 0，仅姿态通道摆动
