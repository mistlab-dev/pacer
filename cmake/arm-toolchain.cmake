# CMake 工具链文件 - ARM Cortex-M7
# 用于交叉编译 STM32H743 项目

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ===== 工具链路径 =====
# Windows 默认路径
if(WIN32)
    set(ARM_TOOLCHAIN_PREFIX "C:/Program Files (x86)/GNU Tools Arm Embedded/9 2019-q4-major/bin")
    if(NOT EXISTS "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc.exe")
        # 备用路径
        set(ARM_TOOLCHAIN_PREFIX "C:/Program Files/GNU Tools Arm Embedded/9 2019-q4-major/bin")
    endif()
else()
    # Linux 使用系统安装的工具链
    set(ARM_TOOLCHAIN_PREFIX "/usr/bin")
endif()

# ===== 编译器 =====
set(CMAKE_C_COMPILER   "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc"     CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-g++"     CACHE FILEPATH "C++ compiler")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc"     CACHE FILEPATH "ASM compiler")
set(CMAKE_LINKER       "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-gcc"     CACHE FILEPATH "Linker")
set(CMAKE_AR           "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-ar"      CACHE FILEPATH "Archiver")
set(CMAKE_OBJCOPY      "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-objcopy" CACHE FILEPATH "Objcopy")
set(CMAKE_OBJDUMP      "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-objdump" CACHE FILEPATH "Objdump")
set(CMAKE_SIZE         "${ARM_TOOLCHAIN_PREFIX}/arm-none-eabi-size"    CACHE FILEPATH "Size")

# ===== 目标架构 =====
set(CPU_PARAMETERS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

# ===== 编译标志 =====
set(CMAKE_C_FLAGS_INIT "${CPU_PARAMETERS} -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_C_FLAGS_DEBUG_INIT "-g3 -O0")
set(CMAKE_C_FLAGS_RELEASE_INIT "-Os -DNDEBUG")

# ===== 链接标志 =====
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_PARAMETERS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,--wrap=_malloc_r -Wl,--wrap=_free_r")

# ===== 不搜索主机系统路径 =====
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ===== 禁用 CMake 内置测试 =====
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)