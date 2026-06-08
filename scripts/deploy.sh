#!/bin/bash
# Pacer 部署脚本 — 从开发机部署到树莓派

set -e

PI_HOST="${1:-raspberrypi.local}"
PI_USER="${2:-pi}"
REMOTE_DIR="/home/${PI_USER}/pacer"

echo "=== Pacer Deploy ==="
echo "Target: ${PI_USER}@${PI_HOST}:${REMOTE_DIR}"

# 1. 同步源码
echo "=== 1. 同步源码 ==="
rsync -avz --exclude build --exclude .git \
    -e ssh ./ "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/"

# 2. 远程编译
echo "=== 2. 远程编译 ==="
ssh "${PI_USER}@${PI_HOST}" << 'EOF'
cd ~/pacer
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
echo "Build OK"
EOF

# 3. 设置 systemd 服务
echo "=== 3. 安装服务 ==="
ssh "${PI_USER}@${PI_HOST}" << 'EOF'
sudo tee /etc/systemd/system/pacer.service > /dev/null << 'SERVICE'
[Unit]
Description=Pacer Balance Bot
After=network.target

[Service]
Type=simple
ExecStartPre=/home/pi/pacer/build/pacer --calibrate
ExecStart=/home/pi/pacer/build/pacer
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable pacer
echo "Service installed. Run: sudo systemctl start pacer"
EOF

echo "=== 部署完成 ==="
