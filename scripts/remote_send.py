#!/usr/bin/env python3
"""PACER 遥控帧发送器 — 无需编译 Go，直连地面 ESP 或飞控 USART3。"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    import subprocess

    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial", "-q"])
    import serial

FRAME_HDR1 = 0xAA
FRAME_HDR2 = 0x55
FRAME_SIZE = 19


def encode_frame(throttle: float, roll: float, pitch: float, yaw: float) -> bytes:
    throttle = max(0.0, min(1.0, throttle))
    roll = max(-1.0, min(1.0, roll))
    pitch = max(-1.0, min(1.0, pitch))
    yaw = max(-1.0, min(1.0, yaw))
    body = struct.pack("<BBffff", FRAME_HDR1, FRAME_HDR2, throttle, roll, pitch, yaw)
    crc = 0
    for b in body:
        crc ^= b
    return body + bytes([crc])


def main() -> int:
    parser = argparse.ArgumentParser(description="发送 PACER 遥控帧")
    parser.add_argument("--port", required=True, help="COM 口（地面 ESP USB 或飞控 USART3）")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=int, default=50)
    parser.add_argument("--duration", type=float, default=10.0, help="发送秒数，0=一直发")
    parser.add_argument("--demo", action="store_true", help="演示波形")
    parser.add_argument("-t", type=float, default=0.0)
    parser.add_argument("-r", type=float, default=0.0)
    parser.add_argument("-p", type=float, default=0.0)
    parser.add_argument("-y", type=float, default=0.0)
    args = parser.parse_args()

    interval = 1.0 / args.hz
    end = time.time() + args.duration if args.duration > 0 else None
    count = 0
    t0 = time.time()

    print(f"发送到 {args.port} @ {args.baud} Hz={args.hz} demo={args.demo}")

    with serial.Serial(args.port, args.baud, timeout=0.1) as sp:
        sp.dtr = False
        sp.rts = False
        while True:
            if end and time.time() >= end:
                break
            t = time.time() - t0
            if args.demo:
                thr, roll, pitch, yaw = 0.0, 0.3 * math.sin(t), 0.2 * math.cos(t * 0.7), 0.0
            else:
                thr, roll, pitch, yaw = args.t, args.r, args.p, args.y
            frame = encode_frame(thr, roll, pitch, yaw)
            sp.write(frame)
            count += 1
            if count % (args.hz * 3) == 0:
                print(f"  tx={count}  T={thr:.2f} R={roll:+.2f} P={pitch:+.2f} Y={yaw:+.2f}")
            time.sleep(interval)

    # 归零
    with serial.Serial(args.port, args.baud, timeout=0.1) as sp:
        sp.write(encode_frame(0, 0, 0, 0))
    print(f"完成，共发送 {count} 帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
