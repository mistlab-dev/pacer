
#!/usr/bin/env python3
"""
树莓派端 IMU 数据发送器 — 配合 imu_viz.py 使用

读取 ICM20948 数据 (通过 pacer 的 --imu-stream 输出解析,
或直接用 smbus2 读寄存器), 通过 UDP 发送给可视化端。

用法 (树莓派上运行):
  python3 imu_sender.py --host <电脑IP>:8888

  或从 pacer --imu-stream 的 stdout 解析:
  sudo ./pacer --imu-stream | python3 imu_sender.py --host <电脑IP>:8888
"""

import sys, socket, json, time, re, argparse, struct

def parse_stream(host_port):
    """解析 pacer --imu-stream 的 stdout, 通过 UDP 发送"""
    host, port = host_port.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, int(port))

    print(f"发送 IMU 数据到 {host}:{port}")
    print("等待 pacer --imu-stream 输入...")

    t0 = time.time()
    for line in sys.stdin:
        # 解析格式: time ax ay az  gx gy gz  Roll Pitch Yaw  Temp
        # 示例: 12.345   0.123  -0.001   9.802   0.50  -0.30   0.10   1.20  -0.50  180.00   35.2
        parts = line.strip().split()
        nums = []
        for p in parts:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if len(nums) >= 10:
            msg = {
                't': nums[0],
                'ax': nums[1], 'ay': nums[2], 'az': nums[3],
                'gx': nums[4], 'gy': nums[5], 'gz': nums[6],
                'roll': nums[7], 'pitch': nums[8], 'yaw': nums[9],
                'temp': nums[10] if len(nums) > 10 else 0,
            }
            sock.sendto(json.dumps(msg).encode(), addr)

def send_binary(host_port):
    """二进制发送 (11 个 float32 = 44 字节)"""
    host, port = host_port.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, int(port))

    print(f"二进制模式 → {host}:{port}")
    t0 = time.time()
    for line in sys.stdin:
        parts = line.strip().split()
        nums = []
        for p in parts:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if len(nums) >= 11:
            # t, ax, ay, az, gx, gy, gz, mx, my, mz, temp
            pkt = struct.pack('<11f', *nums[:11])
            sock.sendto(pkt, addr)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--host', required=True, help='目标地址 IP:PORT')
    p.add_argument('--binary', action='store_true', help='二进制模式')
    args = p.parse_args()

    if args.binary:
        send_binary(args.host)
    else:
        parse_stream(args.host)
