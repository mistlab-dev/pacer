# PACER 依赖文件获取

项目已准备好的核心文件：
- ✅ `cmsis/Include/core_cm7.h` — CMSIS Cortex-M7 核心定义
- ✅ `cmake/startup_stm32h743xx.s` — 启动汇编（手写）
- ✅ `src/hal/system_stm32h7xx.c` — 系统初始化（手写）
- ✅ `freertos/FreeRTOSConfig.h` — FreeRTOS 配置（手写）

缺失的 HAL 和 FreeRTOS 源码（几百个文件），**用 STM32CubeMX 一次性生成最稳**。

---

## 方法 1: STM32CubeMX（推荐）

1. 下载安装 [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx)

2. 新建项目，选择芯片 **STM32H743VI**

3. Pinout & Configuration：
   - **I2C1**: PB6(SCL), PB7(SDA)
   - **TIM1**: CH1(PA8), CH2(PE11), CH3(PE13), CH4(PE14)
   - **USART2**: PA2(TX), PA3(RX) — 调试
   - **USART3**: PD8(TX), PD9(RX) — 遥控

4. Clock Configuration：
   - System Clock: 480MHz

5. Middleware → FREERTOS → Enable

6. Project Manager：
   - Project Name: pacer
   - Toolchain/IDE: **CMake**
   - Code Generator: 勾选 "Generate peripheral initialization..."

7. Generate Code → 得到完整工程

8. 复制生成的目录到 pacer：
   ```
   Drivers/CMSIS/Device/ST/STM32H7xx → stm32hal/cmsis_device/
   Drivers/STM32H7xx_HAL_Driver → stm32hal/hal/
   Middlewares/Third_Party/FreeRTOS/Source → freertos/
   cmake/startup_stm32h743xx.s → 已有，可覆盖
   ```

---

## 方法 2: 手动下载（网络稳定时）

STM32CubeH7 仓库（~100MB）：
```bash
git clone --depth 1 https://github.com/STMicroelectronics/STM32CubeH7.git
```

需要的子目录：
- `Drivers/CMSIS/Device/ST/STM32H7xx/Include/` → cmsis 头文件
- `Drivers/STM32H7xx_HAL_Driver/` → HAL 库
- `Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h743xx.s`

FreeRTOS 官方：
```bash
git clone --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git freertos
```

需要的文件：
- `tasks.c, queue.c, list.c, timers.c`
- `portable/GCC/ARM_CM7/r0p1/port.c, portmacro.h`
- `portable/MemMang/heap_4.c`
- `include/` 所有头文件

---

## 为什么 CubeMX 更好

- HAL 库版本匹配（避免兼容问题）
- 时钟配置自动生成
- FreeRTOS 配置自动适配 Cortex-M7
- 启动文件官方版本（避免手写错误）
- 几百个文件一次性获取，不用逐个下载

---

## 下一步

CubeMX 生成后，更新 CMakeLists.txt 添加源文件路径，然后：

```cmd
scripts\build.bat
scripts\flash.bat
```