#!/usr/bin/env python3
"""PACER 联调监视器 — 同时显示 IMU/姿态 与 USART3 遥控状态。"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    import subprocess

    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial", "-q"])
    import serial
    import serial.tools.list_ports

DEFAULT_BAUD = 105600

IMU_RE = re.compile(
    r"PACER IMU ax=(?P<ax>-?\d+(?:\.\d+)?) ay=(?P<ay>-?\d+(?:\.\d+)?) az=(?P<az>-?\d+(?:\.\d+)?) "
    r"gx=(?P<gx>-?\d+(?:\.\d+)?) gy=(?P<gy>-?\d+(?:\.\d+)?) gz=(?P<gz>-?\d+(?:\.\d+)?) "
    r"roll=(?P<roll>-?\d+(?:\.\d+)?) pitch=(?P<pitch>-?\d+(?:\.\d+)?) yaw=(?P<yaw>-?\d+(?:\.\d+)?) "
    r"T=(?P<temp>-?\d+(?:\.\d+)?)"
)

REMOTE_RE = re.compile(
    r"PACER REMOTE T=(?P<t>-?\d+(?:\.\d+)?) R=(?P<r>-?\d+(?:\.\d+)?) "
    r"P=(?P<p>-?\d+(?:\.\d+)?) Y=(?P<y>-?\d+(?:\.\d+)?) "
    r"conn=(?P<conn>\d) frames=(?P<frames>\d+) drops=(?P<drops>\d+)"
)


def pick_com_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list(serial.tools.list_ports.comports())
    ranked = sorted(
        ports,
        key=lambda p: (
            ("CH347" in (p.description or "").upper()) * 10,
            ("SERIAL-A" in (p.description or "").upper()) * 20,
        ),
        reverse=True,
    )
    if not ranked:
        raise RuntimeError("未找到 COM 口")
    return ranked[0].device


def main() -> int:
    parser = argparse.ArgumentParser(description="PACER IMU + 遥控联调监视器")
    parser.add_argument("--port", help="调试串口 COM（USART1，默认自动选 CH347 SERIAL-A）")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数，0=一直运行")
    args = parser.parse_args()

    port = pick_com_port(args.port)
    print(f"打开 {port} @ {args.baud} (DTR/RTS 关闭)")
    print("等待 PACER IMU / PACER REMOTE ...  Ctrl+C 退出")
    print("-" * 100)

    imu_seq = rc_seq = 0
    last_imu = last_rc = None
    end = time.time() + args.duration if args.duration > 0 else None

    with serial.Serial(port, args.baud, timeout=0.2) as sp:
        sp.dtr = False
        sp.rts = False
        buf = ""
        while True:
            if end and time.time() >= end:
                break
            chunk = sp.read(4096)
            if chunk:
                buf += chunk.decode("utf-8", errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                m = IMU_RE.search(line)
                if m:
                    imu_seq += 1
                    d = {k: float(m[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz", "roll", "pitch", "yaw", "temp")}
                    last_imu = d
                    print(
                        f"[IMU #{imu_seq:04d}] "
                        f"acc {d['ax']:6.2f} {d['ay']:6.2f} {d['az']:6.2f} | "
                        f"gyro {d['gx']:6.3f} {d['gy']:6.3f} {d['gz']:6.3f} | "
                        f"att R{d['roll']:5.1f} P{d['pitch']:5.1f} Y{d['yaw']:5.1f} | "
                        f"T={d['temp']:.1f}"
                    )
                    continue

                m = REMOTE_RE.search(line)
                if m:
                    rc_seq += 1
                    last_rc = m.groupdict()
                    conn = int(last_rc["conn"])
                    print(
                        f"[RC  #{rc_seq:04d}] "
                        f"T={float(last_rc['t']):.2f} R={float(last_rc['r']):+.2f} "
                        f"P={float(last_rc['p']):+.2f} Y={float(last_rc['y']):+.2f} | "
                        f"conn={conn} frames={last_rc['frames']} drops={last_rc['drops']}"
                    )
                    if conn == 0 and rc_seq > 5:
                        print("       [WARN] 遥控未连接 — 检查 ESP-NOW / pacer-remote / USART3 接线")
                    continue

                if line.startswith("PACER "):
                    print(f"[BOOT] {line}")
            time.sleep(0.02)

    print("-" * 100)
    print(f"结束: IMU {imu_seq} 帧, 遥控 {rc_seq} 帧")
    if imu_seq == 0 and rc_seq == 0:
        print("未收到数据。请确认已烧录固件且 BOOT0=0 已 RESET。")
        return 1
    if last_imu and abs(last_imu["az"] - 9.81) < 2.5:
        print("IMU: 平放加速度正常")
    if last_rc and int(last_rc["conn"]) == 1:
        print("遥控: 链路已连接")
    return 0


if __name__ == "__main__":
    sys.exit(main())
