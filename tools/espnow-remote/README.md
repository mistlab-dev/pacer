# PACER ESP-NOW 无线遥控桥

用 **2 个 ESP32**（如 ESP32 Mini）把 `pacer-remote` / USART3 遥控协议无线化，**飞控固件不用改协议**。

## 架构

```
电脑 pacer-remote ──USB──► 地面 ESP32 ──ESP-NOW 2.4G──► 机载 ESP32 ──UART──► STM32 USART3
                              ▲                                              │
                              │                                              ▼
                         scripts/remote_send.py                    USART1 调试口
                         (Python 替代)                              scripts/link_monitor.py
```

帧格式与 `src/remote/remote.c` 相同：19 字节 v2（`0xAA 0x55` + 4×float + XOR）。

## 接线

### 机载 ESP32 ↔ STM32（USART3）

| ESP32 | STM32 |
|-------|-------|
| TX (GPIO17) | PD9 (RX) |
| RX (GPIO16) | PD8 (TX) |
| GND | GND |
| 3.3V | 3.3V |

ESP32 Mini 引脚不同时，改 `air.ino` 里 `UART_TX_PIN` / `UART_RX_PIN`。

### 供电

- 机载 ESP 用 **3.3V**，不要 5V 进 STM32 RX
- 地面 ESP USB 供电

## MAC 配置

1. 复制 `mac_config.h.example` → `mac_config.h`（放在 `air/` 或 `ground/` 目录）
2. 先烧 **air.ino**，串口监视器记下 `Air MAC` → 填入 `ground` 的 `PACER_AIR_MAC`
3. 烧 **ground.ino**，记下 `Ground MAC` → 填入 `air` 的 `PACER_GROUND_MAC`
4. 再烧一次 **air.ino**

## 烧录（Arduino IDE）

1. 板卡：**ESP32 Dev Module**（Mini 通用）
2. 库：**esp32 by Espressif** 2.x+
3. 地面端三种模式：
   - **串口桥**（默认）：电脑 `pacer-remote` 或 `remote_send.py`
   - **演示波形**：编译选项 `-D DEMO_JOYSTICK=1`
   - **ADC 摇杆**：`-D ADC_JOYSTICK=1`，GPIO34/35/32/33 接电位器

## 联调步骤

### 1. 仅 ESP-NOW（不接飞控）

地面端 `-D DEMO_JOYSTICK=1`，机载端串口应每 2 秒打印 `espnow_rx=` 增加。

### 2. 接 STM32

飞控固件保持 `CFG_REMOTE_DEBUG=1`，调试口运行：

```bat
scripts\link_monitor.bat
```

另一终端向**地面 ESP 的 COM** 发遥控：

```bat
python scripts\remote_send.py --port COMx --demo
```

或：

```bat
tools\pacer-remote\pacer-remote.exe -port COMx -demo
```

调试口应出现 `PACER REMOTE ... conn=1 frames=...`。

### 3. 飞控模式

IMU + 遥控验证通过后，`config.h` 设 `CFG_UART_PLAIN_DEBUG=0`，重新烧录，进入 FreeRTOS 飞控。

## 注意

- ESP-NOW 用 2.4GHz，与成品 RC 接收机保持距离
- 首次联调 1～2 米内测试
- 飞控有丢链保护，台架先测，**拆桨**
