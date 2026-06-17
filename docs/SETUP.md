# PACER 环境配置指南 (Windows 11)

本文档记录所有**需要手动操作**的步骤。按顺序完成即可。

---

## 第一步：安装工具链

### 1.1 GNU Arm Embedded Toolchain（交叉编译器）

1. 打开 https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
2. 找到最新版本，下载 Windows 版安装包
   - 文件名类似：`arm-gnu-toolchain-10.3-2021.10-mingw-w64-i686-arm-none-eabi.exe`
3. 双击运行安装程序
4. 安装路径建议：`C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\`
   - ⚠️ **记住这个路径**，编译脚本里要用
5. 安装到最后一步，**勾选 "Add path to environment variable"**
6. 验证：
   ```cmd
   arm-none-eabi-gcc --version
   ```

如果安装时没勾选 PATH，手动添加：
1. 按 `Win + S` 搜索「环境变量」→ 点击「编辑系统环境变量」
2. 点击「环境变量」按钮
3. 在「系统变量」中找到 `Path`，双击
4. 点击「新建」，粘贴 bin 目录路径
   - 例如 `C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin`
5. 确定 → 确定 → 确定，**重启终端**

---

### 1.2 CMake（构建系统）

1. 打开 https://cmake.org/download/
2. 下载 Windows x64 Installer：`cmake-x.x.x-windows-x64.msi`
3. 双击运行
4. 安装选项中**选择 "Add CMake to the system PATH for all users"**
   - ⚠️ 这一步很重要，否则命令行找不到 cmake
5. 一路 Next 完成安装
6. 验证：
   ```cmd
   cmake --version
   ```

---

### 1.3 STM32CubeMX（代码生成工具）

1. 打开 https://www.st.com/en/development-tools/stm32cubemx
2. 需要**注册/登录 ST 账号**（免费）
3. 点击 **Get Software**，下载 `.exe` 安装包
   - 文件名类似：`SetupSTM32CubeMX-6.12.0.exe`
4. 双击运行，安装路径保持默认
5. 首次运行需要下载 STM32H7 固件包：
   - 打开 CubeMX → 菜单栏 **Help → Manage embedded software packages**
   - 找到 **STM32H7** 系列，勾选最新版本
   - 点击 **Install Now**，等待下载（约 500MB）
   - ⚠️ **必须完成**，否则无法生成 H743 代码

---

### 1.4 STM32CubeProgrammer（烧录工具）

1. 打开 https://www.st.com/en/development-tools/stm32cubeprog.html
2. 点击 **Get Software**，下载 Windows 版 `.exe`
   - 文件名类似：`SetupSTM32CubeProgrammer-2.17.0.exe`
3. 双击运行安装，路径保持默认
4. 安装完成后 ST-Link USB 驱动会自动安装
5. 验证：
   - 插上 ST-Link 调试器
   - 打开 STM32CubeProgrammer
   - 左侧选 ST-LINK，点 Refresh，应能看到设备

---

### 1.5 Go 语言（遥控上位机，可选）

1. 打开 https://go.dev/dl/
2. 下载 Windows 版：`go1.22.x.windows-amd64.msi`
3. 双击安装，默认路径 `C:\Program Files\Go\`
4. 安装程序会自动添加 PATH
5. 验证：
   ```cmd
   go version
   ```

---

### 1.6 安装验证

打开 **PowerShell** 或 **CMD**，逐个执行：

```cmd
arm-none-eabi-gcc --version
cmake --version
go version
```

STM32CubeMX 和 STM32CubeProgrammer 能在开始菜单找到快捷方式即安装成功。

---

## 第二步：STM32CubeMX 生成依赖文件

这是**最关键的一步**，生成 HAL 库和 FreeRTOS 源码。

### 2.1 创建新项目

1. 打开 STM32CubeMX
2. **File → New Project**
3. 搜索 `STM32H743VI`（或 `STM32H743VGTx`）
4. 双击选中，进入 Pinout 配置界面

### 2.2 Pinout & Configuration 配置

按以下表格配置引脚：

| 功能 | 引脚 | 配置方式 |
|------|------|----------|
| **I2C1_SCL** | PB6 | Connectivity → I2C1 → Mode: I2C → PB6 自动分配 |
| **I2C1_SDA** | PB7 | 同上，PB7 自动分配 |
| **TIM1_CH1** | PA8 | Timers → TIM1 → Channel1: PWM Generation CH1 |
| **TIM1_CH2** | PE11 | Timers → TIM1 → Channel2: PWM Generation CH2 |
| **TIM1_CH3** | PE13 | Timers → TIM1 → Channel3: PWM Generation CH3 |
| **TIM1_CH4** | PE14 | Timers → TIM1 → Channel4: PWM Generation CH4 |
| **USART2_TX** | PA2 | Connectivity → USART2 → Mode: Asynchronous |
| **USART2_RX** | PA3 | 同上，自动分配 |
| **USART3_TX** | PD8 | Connectivity → USART3 → Mode: Asynchronous → PD8 手动选 |
| **USART3_RX** | PD9 | 同上，PD9 手动选 |

**注意事项：**
- TIM1 是高级定时器，需要在 BDTR 寄存器中使能 Main Output Enable (MOE)
- USART3 的 PD8/PD9 可能需要手动在引脚图上点击选择

### 2.3 Clock Configuration 配置

1. 点击顶部 **Clock Configuration** 标签
2. 目标：System Core Clock = **480 MHz**

CubeMX 会自动计算分频系数，确认最终 SYSCLK 为 480 MHz 即可。

如果有外部晶振 (HSE 8MHz)：
```
HSE (8 MHz) → PLL1 → SYSCLK (480 MHz)
```

如果用内部振荡器 (HSI 64MHz)：
```
HSI (64 MHz) → PLL1 → SYSCLK (480 MHz)
```

### 2.4 Middleware → FreeRTOS 配置

1. 点击左侧 **Middleware → FREERTOS**
2. **Interface**: 选 `CMSIS_V2`
3. **Config parameters**:
   - `TOTAL_HEAP_SIZE`: `80000`
   - `MAX_PRIORITIES`: `7`
   - 其他保持默认
4. **Tasks and Queues**:
   - 默认的 `defaultTask` 可以删除或保留
   - 我们在代码中自己创建任务（ctrl_task, rx_task, telem_task）

### 2.5 Project Manager 设置

1. 点击顶部 **Project Manager** 标签
2. **Project Settings**:
   - Project Name: `pacer`
   - Project Location: 选一个临时目录（如 `D:\temp\pacer-cube`）
   - Toolchain/IDE: **CMake** ← 重要！
3. **Code Generator**:
   - ✅ Generate peripheral initialization as a pair of '.c/.h' files
   - ✅ Keep User Code when re-generating
   - ✅ Delete previously generated files when not re-generated

### 2.6 生成代码

1. 菜单 **Project → Generate Code**（快捷键 `Ctrl+Shift+G`）
2. 等待生成完成
3. 打开生成目录，确认结构：

```
D:\temp\pacer-cube\
├── CMakeLists.txt
├── cmake\
│   └── startup_stm32h743xx.s
├── Core\
│   ├── Inc\
│   │   ├── main.h
│   │   ├── stm32h7xx_hal_conf.h    ← 需要
│   │   ├── stm32h7xx_it.h
│   │   └── FreeRTOSConfig.h        ← 需要
│   └── Src\
│       ├── main.c
│       ├── stm32h7xx_it.c          ← 需要
│       ├── stm32h7xx_hal_msp.c     ← 需要
│       └── freertos.c
├── Drivers\
│   ├── CMSIS\
│   │   └── Device\ST\STM32H7xx\    ← 需要
│   └── STM32H7xx_HAL_Driver\       ← 需要
└── Middlewares\
    └── Third_Party\FreeRTOS\Source\  ← 需要
```

---

## 第三步：复制文件到 pacer 项目

假设：
- CubeMX 生成目录：`D:\temp\pacer-cube\`
- pacer 项目目录：`D:\work\pacer\`

### 3.1 一键复制脚本

在 PowerShell 中运行（路径按实际情况修改）：

```powershell
$CUBE = "D:\temp\pacer-cube"
$PACER = "D:\work\pacer"

# 创建目录
New-Item -ItemType Directory -Force -Path "$PACER\stm32hal\cmsis_device\Include"
New-Item -ItemType Directory -Force -Path "$PACER\stm32hal\hal\Inc"
New-Item -ItemType Directory -Force -Path "$PACER\stm32hal\hal\Src"
New-Item -ItemType Directory -Force -Path "$PACER\freertos"

# CMSIS 设备头文件
Copy-Item "$CUBE\Drivers\CMSIS\Device\ST\STM32H7xx\Include\*" "$PACER\stm32hal\cmsis_device\Include\" -Recurse -Force

# HAL 库
Copy-Item "$CUBE\Drivers\STM32H7xx_HAL_Driver\Inc\*" "$PACER\stm32hal\hal\Inc\" -Recurse -Force
Copy-Item "$CUBE\Drivers\STM32H7xx_HAL_Driver\Src\*" "$PACER\stm32hal\hal\Src\" -Recurse -Force

# FreeRTOS 源码
Copy-Item "$CUBE\Middlewares\Third_Party\FreeRTOS\Source\*" "$PACER\freertos\" -Recurse -Force

# HAL 配置 + FreeRTOS 配置
Copy-Item "$CUBE\Core\Inc\stm32h7xx_hal_conf.h" "$PACER\include\" -Force
Copy-Item "$CUBE\Core\Inc\FreeRTOSConfig.h" "$PACER\freertos\" -Force

# 中断处理 + Msp 初始化
Copy-Item "$CUBE\Core\Src\stm32h7xx_it.c" "$PACER\src\hal\" -Force
Copy-Item "$CUBE\Core\Src\stm32h7xx_hal_msp.c" "$PACER\src\hal\" -Force

# 启动文件（覆盖手写版本）
Copy-Item "$CUBE\cmake\startup_stm32h743xx.s" "$PACER\cmake\" -Force

Write-Host "Done! All files copied."
```

### 3.2 复制后检查

```powershell
# 确认关键文件存在
Test-Path "$PACER\stm32hal\cmsis_device\Include\stm32h743xx.h"
Test-Path "$PACER\stm32hal\hal\Src\stm32h7xx_hal_i2c.c"
Test-Path "$PACER\freertos\tasks.c"
Test-Path "$PACER\include\stm32h7xx_hal_conf.h"
```

全部返回 `True` 即复制成功。

---

## 第四步：更新 CMakeLists.txt

复制完成后，更新 `CMakeLists.txt` 添加新的源文件路径。

打开 `D:\work\pacer\CMakeLists.txt`，替换 `PACER_SOURCES` 和 `include_directories`：

```cmake
# 头文件搜索路径
include_directories(
    include
    cmsis/Include
    stm32hal/cmsis_device/Include
    stm32hal/hal/Inc
    freertos/include
    freertos/portable/GCC/ARM_CM7/r0p1
)

# 源文件
set(PACER_SOURCES
    # 启动文件
    cmake/startup_stm32h743xx.s

    # 入口
    src/main.c
    src/hal/usart_printf.c
    src/hal/stm32h7xx_it.c
    src/hal/stm32h7xx_hal_msp.c

    # HAL 层
    src/hal/hal_i2c.c
    src/hal/hal_gpio.c
    src/hal/hal_tim.c
    src/hal/system_stm32h7xx.c

    # STM32 HAL 源码
    stm32hal/hal/Src/stm32h7xx_hal.c
    stm32hal/hal/Src/stm32h7xx_hal_cortex.c
    stm32hal/hal/Src/stm32h7xx_hal_rcc.c
    stm32hal/hal/Src/stm32h7xx_hal_rcc_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_gpio.c
    stm32hal/hal/Src/stm32h7xx_hal_i2c.c
    stm32hal/hal/Src/stm32h7xx_hal_i2c_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_tim.c
    stm32hal/hal/Src/stm32h7xx_hal_tim_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_uart.c
    stm32hal/hal/Src/stm32h7xx_hal_uart_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_pwr.c
    stm32hal/hal/Src/stm32h7xx_hal_pwr_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_flash.c
    stm32hal/hal/Src/stm32h7xx_hal_flash_ex.c
    stm32hal/hal/Src/stm32h7xx_hal_dma.c
    stm32hal/hal/Src/stm32h7xx_hal_dma_ex.c

    # FreeRTOS
    freertos/tasks.c
    freertos/queue.c
    freertos/list.c
    freertos/timers.c
    freertos/portable/GCC/ARM_CM7/r0p1/port.c
    freertos/portable/MemMang/heap_4.c

    # 传感器
    src/sensor/imu.c
    src/sensor/impl/imu_icm20948.c

    # 滤波
    src/filter/filter.c

    # 控制
    src/ctrl/pid.c
    src/ctrl/attitude.c
    src/ctrl/quad_mixer.c

    # 电机
    src/motor/motor.c

    # 遥控
    src/remote/remote.c

    # 应用
    src/app/app.c
)
```

---

## 第五步：编译

打开 **PowerShell** 或 **CMD**：

```cmd
cd D:\work\pacer
scripts\build.bat
```

编译成功输出：
```
===== Build Complete =====
Output: D:\work\pacer\build\pacer.elf
Binary: D:\work\pacer\build\pacer.bin
   text	   data	    bss	    dec	    hex	filename
  45231	    128	  32800	  78159	  1312f	D:\work\pacer\build\pacer.elf
```

**常见编译错误：**

| 错误 | 解决方法 |
|------|----------|
| `stm32h7xx_hal_conf.h not found` | 检查 `include/` 目录是否复制了该文件 |
| `FreeRTOSConfig.h not found` | 检查 `freertos/` 目录结构 |
| `core_cm7.h not found` | 检查 `cmsis/Include/` 路径 |
| `undefined reference to HAL_xxx` | 检查 HAL .c 文件是否添加到 CMakeLists.txt |
| `multiple definition of SystemInit` | 删除 `src/hal/system_stm32h7xx.c`，用 CubeMX 版本 |
| `arm-none-eabi-gcc not found` | 工具链 PATH 没配好，参见第一步 |

---

## 第六步：烧录

### 硬件连接

ST-Link 与 STM32H743VI 接线：

| ST-Link | STM32H743 | 说明 |
|---------|-----------|------|
| SWDIO | PA13 | SWD 数据 |
| SWCLK | PA14 | SWD 时钟 |
| GND | GND | 地 |
| 3.3V | 3.3V | 可选，给目标板供电 |

### 烧录

```cmd
cd D:\work\pacer
scripts\flash.bat
```

全片擦除后烧录：
```cmd
scripts\flash.bat erase
```

烧录成功后 STM32 自动复位运行。

---

## 第七步：遥控上位机

### 编译

```cmd
cd D:\work\pacer\tools\pacer-remote
go build
```

生成 `pacer-remote.exe`。

### 使用

```cmd
# 连接飞控（COM 口号在设备管理器中查看）
pacer-remote.exe -port COM3 -baud 115200 -t 0.5

# Demo 模式（自动摇摆，测试飞控响应）
pacer-remote.exe -port COM3 -demo

# 指定初始值
pacer-remote.exe -port COM3 -t 0.3 -r 0.1 -p -0.1
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `-port` | 必填 | 串口号（设备管理器中查看，如 COM3） |
| `-baud` | 115200 | 波特率 |
| `-freq` | 50 | 发送频率 Hz |
| `-t` | 0 | 油门 0~1 |
| `-r` | 0 | 滚转 -1~+1 |
| `-p` | 0 | 俯仰 -1~+1 |
| `-y` | 0 | 偏航 -1~+1 |
| `-demo` | false | 自动摇摆模式 |

⚠️ **首次测试请拆掉螺旋桨！**

---

## 常见问题

### CubeMX 生成的代码和 pacer 现有代码冲突？

保留 pacer 的 `src/main.c` 和 `src/app/app.c`，只复制 HAL/FreeRTOS/CMSIS 部分。CubeMX 生成的 `Core/Src/main.c` 不需要。

### 编译报 `multiple definition of SystemInit`？

CubeMX 生成的 `system_stm32h7xx.c` 和手写版本冲突，删除 `src/hal/system_stm32h7xx.c`，用 CubeMX 版本。

### FreeRTOS 任务不运行？

检查 `FreeRTOSConfig.h` 中的 `configTOTAL_HEAP_SIZE`，太小会导致任务创建失败（建议 80000）。

### ESC 不转动？

1. TIM1 确认配置为 PWM 模式
2. BDTR 寄存器的 MOE 位必须置位（高级定时器特有）
3. ESC 解锁需要最低油门（1000μs）保持 2 秒以上
4. 检查电机线序是否对应正确通道

### build.bat 报工具链找不到？

编辑 `scripts\build.bat`，修改 `ARM_TOOLCHAIN` 变量为你的实际安装路径：
```cmd
set ARM_TOOLCHAIN=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin
```

### flash.bat 报 CubeProgrammer 找不到？

编辑 `scripts\flash.bat`，修改 `STLINK_PATH` 变量：
```cmd
set STLINK_PATH=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin
```

---

## 完成检查清单

- [ ] ARM 工具链安装完成，`arm-none-eabi-gcc --version` 正常
- [ ] CMake 安装完成，`cmake --version` 正常
- [ ] STM32CubeMX 安装完成，STM32H7 固件包已下载
- [ ] STM32CubeProgrammer 安装完成
- [ ] Go 安装完成（可选），`go version` 正常
- [ ] CubeMX 项目配置完成（I2C1/TIM1/USART2/USART3/FreeRTOS）
- [ ] 代码生成完成
- [ ] HAL/CMSIS/FreeRTOS 文件复制到 pacer 项目
- [ ] CMakeLists.txt 更新完成
- [ ] 编译成功，生成 pacer.bin
- [ ] 烧录成功
- [ ] 遥控上位机测试完成

---

全部完成后，后续开发只需修改 `src/` 下的飞控代码，不再依赖 CubeMX。
