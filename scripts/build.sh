#!/bin/bash
# PACER 编译脚本 - Linux
# STM32H743VI + FreeRTOS

set -e

# ===== 工具链检测 =====
ARM_GCC=$(which arm-none-eabi-gcc 2>/dev/null || echo "")
if [ -z "$ARM_GCC" ]; then
    echo "[ERROR] arm-none-eabi-gcc not found"
    echo "Install: sudo apt install gcc-arm-none-eabi"
    exit 1
fi

CMAKE=$(which cmake 2>/dev/null || echo "")
if [ -z "$CMAKE" ]; then
    echo "[ERROR] cmake not found"
    echo "Install: sudo apt install cmake"
    exit 1
fi

# ===== 项目路径 =====
PROJECT_ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR="$PROJECT_ROOT/build"

# ===== 参数解析 =====
CLEAN=0
TARGET="Debug"
while [ $# -gt 0 ]; do
    case "$1" in
        clean)   CLEAN=1 ;;
        release) TARGET="Release" ;;
        *)       echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

echo "===== PACER Build Script ====="
echo "Target: $TARGET"
echo "Project: $PROJECT_ROOT"

# ===== 清理 =====
if [ $CLEAN -eq 1 ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# ===== 创建构建目录 =====
mkdir -p "$BUILD_DIR"

# ===== CMake 配置 =====
echo "Configuring with CMake..."
cd "$BUILD_DIR"
cmake -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE="$TARGET" \
    -DCMAKE_TOOLCHAIN_FILE="$PROJECT_ROOT/cmake/arm-toolchain.cmake" \
    "$PROJECT_ROOT"

# ===== 编译 =====
echo "Building..."
cmake --build . --config "$TARGET"

echo "===== Build Complete ====="
echo "Output: $BUILD_DIR/pacer.elf"
echo "Binary: $BUILD_DIR/pacer.bin"
echo ""

# ===== 显示大小 =====
arm-none-eabi-size "$BUILD_DIR/pacer.elf"

cd "$PROJECT_ROOT"