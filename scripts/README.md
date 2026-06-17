# PACER 编译烧录脚本

## Windows 环境

### 1. 安装工具链

| 工具 | 版本 | 下载地址 |
|------|------|----------|
| **GNU Arm Embedded Toolchain** | 9-2019-q4-major 或更高 | https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-arm |
| **CMake** | 3.15+ | https://cmake.org/download/ |
| **STM32CubeProgrammer** | 任意版本 | https://www.st.com/en/development-tools/stm32cubeprog.html |
| **STM32CubeMX** | 任意版本（生成 HAL/FreeRTOS） | https://www.st.com/en/development-tools/stm32cubemx |

### 2. 脚本使用

```cmd
REM 编译（Debug）
scripts\build.bat

REM 编译（Release）
scripts\build.bat release

REM 清理后编译
scripts\build.bat clean

REM 烧录
scripts\flash.bat

REM 全片擦除后烧录
scripts\flash.bat erase
```

### 3. 输出文件

| 文件 | 位置 | 用途 |
|------|------|------|
| `pacer.elf` | `build/` | 调试符号，用于 GDB |
| `pacer.bin` | `build/` | 烧录用，原始二进制 |
| `pacer.hex` | `build/` | 部分 ST-Link 工具偏好 hex |

---

## Linux 环境

### 1. 安装工具链

```bash
# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi cmake stlink-tools

# 或手动安装 ARM 工具链
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-arm
```

### 2. 脚本使用

```bash
# 编译（Debug）
./scripts/build.sh

# 编译（Release）
./scripts/build.sh release

# 清理后编译
./scripts/build.sh clean

# 烧录（需 ST-Link 硬件）
./scripts/flash.sh

# 全片擦除后烧录
./scripts/flash.sh erase
```

---

## 硬件连接

ST-Link 与 STM32H743VI 连接：

| ST-Link | STM32H743 |
|---------|-----------|
| SWDIO   | PA13      |
| SWCLK   | PA14      |
| GND     | GND       |
| 3.3V    | 3.3V (可选，目标供电) |

---

## 常见问题

### Q: 编译报 `arm-none-eabi-gcc not found`

修改 `scripts/build.bat` 中的 `ARM_TOOLCHAIN` 变量为实际安装路径。

### Q: 烧录报 `ST-Link not found`

1. 检查 ST-Link USB 连接
2. 检查驱动安装（Windows 可能需要 ST-Link 驱动）
3. 在 STM32CubeProgrammer 中测试 ST-Link 连接

### Q: 烧录后程序不运行

1. 检查 BOOT0 引脚接地（正常启动模式）
2. 检查 NRST 复位引脚
3. 用 STM32CubeProgrammer 手动复位后观察

---

## 从 CubeMX 生成依赖文件

`cmsis/` `stm32hal/` `freertos/` 目录需要从 STM32CubeMX 生成：

1. 新建项目，选择 **STM32H743VI**
2. 配置引脚：
   - I2C1: PB6(SCL), PB7(SDA)
   - TIM1 CH1(PA8), CH2(PE11), CH3(PE13), CH4(PE14)
   - USART2: PA2(TX), PA3(RX)
   - USART3: PD8(TX), PD9(RX)
3. Middleware → **FREERTOS** → Enable
4. Project Manager → Toolchain/IDE: **CMake**
5. Code Generator → 勾选 "Generate peripheral initialization..."
6. Generate Code → 得到完整工程目录

将生成的 `Drivers/CMSIS` `Drivers/STM32H7xx_HAL_Driver` `Middlewares/Third_Party/FreeRTOS` 复制到 pacer 项目对应目录。