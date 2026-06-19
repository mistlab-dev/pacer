# PACER ESP-NOW 无线遥控桥

用 **2 个 ESP32** 把现有 `pacer-remote` / USART3 遥控协议无线化，**飞控固件不用改协议**。

## 架构

```
电脑 pacer-remote ──USB──► 地面 ESP32 ──ESP-NOW 2.4G──► 机载 ESP32 ──UART──► STM32 USART3
```

帧格式与 `src/remote/remote.c` 相同：19 字节 v2（`0xAA 0x55` + 4×float + XOR）。

## 采购

| 数量 | 型号 |
|------|------|
| 2 | ESP32 开发板（DevKitC / ESP32-C3 等，带 USB） |

## 接线

### 机载 ESP32 ↔ STM32（USART3）

| ESP32 | STM32 |
|-------|-------|
| TX (GPIO17) | PD9 (RX) |
| RX (GPIO16) | PD8 (TX) |
| GND | GND |

引脚可在 `air.ino` 里改 `UART_TX_PIN` / `UART_RX_PIN`。

### 供电

- 机载 ESP 用 3.3V（飞控 3.3V 或独立 LDO），**不要接 5V 到 STM32 RX**。
- 地面 ESP 用 USB 供电即可。

## 烧录步骤

1. 安装 [Arduino IDE](https://www.arduino.cc/en/software) + 板卡 **esp32 by Espressif**
2. 先烧 **air/air.ino**，打开串口监视器，记下 `Air MAC`
3. 把 MAC 填进 **ground/ground.ino** 的 `airMac[]`，烧地面端，记下 `Ground MAC`
4. 把 Ground MAC 填回 **air.ino** 的 `groundMac[]`，再烧一次机载端
5. 飞机 **BOOT0=0**，STM32 正常运行

## 使用

```bash
# 地面 ESP 的 USB 在设备管理器里是 COMx
pacer-remote.exe -port COMx -baud 115200 -demo
```

`-demo` 会持续发摇杆帧 → 经地面 ESP → ESP-NOW → 机载 ESP → STM32。

## 以后：纯手柄

在 `ground.ino` 里读 ADC 摇杆组帧，或把 `DEMO_JOYSTICK` 改成 1 做链路测试，不再依赖电脑。

## 注意

- ESP-NOW 用 **2.4GHz WiFi 信道**，与成品 RC 接收机不要同时贴在一起干扰测试。
- 首次联调距离 1～2 米，确认 `fwd=` 计数增加。
- 飞控侧已有遥控超时保护（丢链自动降落逻辑），务必在台架上先测。
