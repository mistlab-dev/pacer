#!/bin/bash
# PACER 烧录脚本 - Linux
# STM32H743VI via ST-Link (st-flash)

set -e

# ===== 工具检测 =====
ST_FLASH=$(which st-flash 2>/dev/null || echo "")
if [ -z "$ST_FLASH" ]; then
    echo "[ERROR] st-flash not found"
    echo "Install: sudo apt install stlink-tools"
    echo "Or: git clone https://github.com/stlink-org/stlink && make && sudo make install"
    exit 1
fi

# ===== 项目路径 =====
PROJECT_ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR="$PROJECT_ROOT/build"
BIN_FILE="$BUILD_DIR/pacer.bin"

# ===== 检查固件 =====
if [ ! -f "$BIN_FILE" ]; then
    echo "[ERROR] Binary not found: $BIN_FILE"
    echo "Please run build.sh first."
    exit 1
fi

# ===== 参数 =====
ERASE=0
while [ $# -gt 0 ]; do
    case "$1" in
        erase) ERASE=1 ;;
        *)     ;;
    esac
    shift
done

echo "===== PACER Flash Script ====="
echo "Target: STM32H743VI"
echo "Binary: $BIN_FILE"
echo ""

# ===== 全片擦除（可选）=====
if [ $ERASE -eq 1 ]; then
    echo "Erasing entire chip..."
    st-flash erase
fi

# ===== 烧录 =====
echo "Flashing firmware..."
st-flash write "$BIN_FILE" 0x08000000

if [ $? -ne 0 ]; then
    echo "[ERROR] Flash failed"
    exit 1
fi

echo "===== Flash Complete ====="
echo "Firmware running..."
echo ""