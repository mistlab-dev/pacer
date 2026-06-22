# 鹿小班 STM32H743 空板联调记录

> 板型：鹿小班 H743ZIT6 + 板载 CH347（USB DFU 烧录 + 双串口）  
> 更新：2026-06

## 硬件与 COM 口

| 项目 | 说明 |
|------|------|
| USB | DFU 烧录（BOOT0=1 + RESET） |
| COM3 | CH347 **SERIAL-A**，对应板载 **UART0** → MCU **USART1 (PA9/PA10)** |
| COM4 | CH347 SERIAL-B，本板空板时无输出 |
| 状态灯 | 电源红灯常亮 ≠ 程序灯；用户 LED 多在 PE1，核心板常未接 |

## 已确认问题与修复

### 1. 波特率：名义 115200，实测约 105600

- **现象**：COM3 有稳定字节流，115200 下全文乱码；宽范围扫波特率在 **100800～108000** 可读出固定明文。
- **比例**：`105600 / 115200 = 11/12`，为系统性时钟偏差，不是接线错误。
- **原因**：HSI + PLL 无外部晶振时，USART 外设时钟与 HAL 按 115200 计算的分频不完全一致。
- **修复（当前）**：固件 `CFG_DEBUG_UART_BAUD` 保持 **115200**；PC 串口选 **105600 8N1**。
- **勿**把固件也改成 105600（会叠一层偏差，线速约 96800，PC 需 98400）。
- **根治（后续）**：板载 HSE 晶振 + CubeMX/时钟树用 HSE；或示波器/逻辑分析仪校准 BRR。

### 2. 空板无 IMU → I2C 阻塞 → 看门狗复位

- **现象**：串口反复打印 `PACER BOOT` … `PACER RUN`，看不到 `PACER ALIVE`。
- **原因**：`imu_read()` 在无传感器时长时间阻塞 I2C；看门狗 500ms 超时复位。
- **修复**：
  - `imu_init` 失败时 `g.imu_ok = false`，控制任务 **跳过** `imu_read()`；
  - 看门狗在控制循环 **入口** 先 `watchdog_kick()`。

### 3. 乱码曾误判为 printf 问题

- nano.specs 需 `-Wl,-u,_printf_float` 才能打 `%.f`（已加链接选项）。
- 多任务 `printf` 需串口互斥（`uart_write_bytes` + mutex，已加）。
- **空板排查阶段**应优先 **固定明文字符串** + **扫波特率**，再恢复格式化日志。
- 配置开关：`CFG_UART_PLAIN_DEBUG = 1` 时仅 `uart_puts` 固定串、关闭 CLI 与各模块浮点 `printf`。

## 烧录与看日志流程

```
烧录：BOOT0=1 → RESET → DFU 烧 build/pacer.bin @ 0x08000000
运行：BOOT0=0 → RESET
串口：COM3，105600 8N1
```

脚本：

```bat
scripts\flash_and_check.bat   REM 等 DFU → 烧录 → serial_check
python scripts\serial_check.py
```

## 预期串口输出（PLAIN_DEBUG=1）

```text
PACER BOOT
PACER INIT START
PACER NO IMU
PACER INIT OK
PACER RUN
PACER ALIVE
PACER ALIVE
...
```

## 配置索引（config.h）

| 宏 | 当前值 | 说明 |
|----|--------|------|
| `CFG_DEBUG_UART_PORT` | `PACER_DEBUG_USART1` | 鹿小班 UART0 |
| `CFG_DEBUG_UART_BAUD` | `115200`（固件） | PC 用 **105600**；勿改固件为 105600 |
| `CFG_UART_PLAIN_DEBUG` | `1` | 排查用，且**关闭 IWDG**；稳定后改 `0` |

## 后续开发 TODO

- [ ] 烧录含 105600 + 看门狗修复的最新固件，确认 `PACER ALIVE` 稳定 1Hz
- [ ] 接 IMU 后关闭 `CFG_UART_PLAIN_DEBUG`，验证浮点遥测与 CLI
- [ ] 评估 HSE 时钟方案，争取 PC/固件均用 115200
- [ ] USART3 遥控口是否同样偏差（若无线桥接异常可同步扫波特率）
- [ ] 提交当前改动（USART1、脚本、BRINGUP、printf 链接）

## 排查教训（简）

1. 有流量 ≠ 波特率对；115200 读不到关键字 → **扫波特率**。
2. 先 **固定 ASCII**，再恢复 `printf` / 浮点 / 多任务日志。
3. 启动日志重复 → 查 **看门狗** 与 **阻塞外设（I2C）**。
4. DFU 模式下 COM 口无应用日志，属正常。
