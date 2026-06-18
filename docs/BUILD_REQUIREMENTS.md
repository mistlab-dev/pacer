# PACER 编译依赖

## 问题

OpenCloudOS 只打包了 `arm-none-eabi-gcc-cs`（编译器），缺少 `newlib`（嵌入式 C 标准库）。

编译报错：
```
fatal error: stdint.h: No such file or directory
```

检查：
```bash
ls /usr/arm-none-eabi/include/  # 不存在
yum search newlib               # 无结果
```

## 解决方案

### 方案 1：ARM 官方工具链（推荐）

下载 ARM GNU Toolchain 并替换 CMake 路径：

```bash
# 1. 下载
cd /opt
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz
tar xf arm-gnu-toolchain-*.tar.xz
ln -s arm-gnu-toolchain-*/ arm-none-eabi

# 2. 修改 CMakeLists.txt 的路径
# -B/opt/arm-none-eabi/bin → -B/opt/arm-none-eabi/bin
# CMAKE_C_FLAGS_INIT → 指向新工具链
```

或直接用环境变量：
```bash
export PATH=/opt/arm-none-eabi/bin:$PATH
cd /root/work/pacer/build
cmake -DCMAKE_C_COMPILER=arm-none-eabi-gcc ..
make
```

### 方案 2：手动安装 newlib

从 Fedora/CentOS 源下载 RPM：

```bash
# Fedora 有这个包
yum install --downloadonly arm-none-eabi-newlib  # (需要 Fedora 源)
# 或手动下载 RPM 安装
rpm -ivh arm-none-eabi-newlib-*.rpm
```

安装后验证：
```bash
ls /usr/arm-none-eabi/include/stdint.h  # 应存在
```

## 编译步骤

依赖解决后：

```bash
cd /root/work/pacer
mkdir -p build && cd build
cmake ..
make
```

生成文件：
- `pacer.elf` — 可执行
- `pacer.bin` — 烧录镜像
- `pacer.hex` — Intel HEX

## 烧录

STM32H743VI 用 ST-Link 或 OpenOCD：

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program pacer.elf verify reset exit"

st-flash write pacer.bin 0x08000000
```

## 当前修复进度

| Commit | 内容 |
|--------|------|
| `fb82ddd` | clock PLL、重复符号、DWT overflow、ISR safety |
| `a06c22a` | `_sbrk` 堆边界、链接脚本双重堆、遥控注释 |
| `d045848` | 自动降落、起飞渐变、解锁延时、异常处理 |
| `87dd56b` | MSP 整理：I2C/ADC 初始化集中 |
| `35e80a3` | CMake ARM 工具链路径 |
| `2e397d2` | CLI 串口在线调参 |

代码修复完毕，依赖需手动解决。