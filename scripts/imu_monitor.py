#!/usr/bin/env python3
"""PACER IMU 串口监视器 — 在 PC 上实时显示 IMU/姿态并做简单健康检查。"""

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
READ_TIMEOUT = 0.2

IMU_RE = re.compile(
    r"PACER IMU ax=(?P<ax>-?\d+(?:\.\d+)?) ay=(?P<ay>-?\d+(?:\.\d+)?) az=(?P<az>-?\d+(?:\.\d+)?) "
    r"gx=(?P<gx>-?\d+(?:\.\d+)?) gy=(?P<gy>-?\d+(?:\.\d+)?) gz=(?P<gz>-?\d+(?:\.\d+)?) "
    r"roll=(?P<roll>-?\d+(?:\.\d+)?) pitch=(?P<pitch>-?\d+(?:\.\d+)?) yaw=(?P<yaw>-?\d+(?:\.\d+)?) "
    r"T=(?P<temp>-?\d+(?:\.\d+)?)"
)

BIAS_RE = re.compile(
    r"PACER IMU BIAS gx=(?P<gx>-?\d+(?:\.\d+)?) gy=(?P<gy>-?\d+(?:\.\d+)?) gz=(?P<gz>-?\d+(?:\.\d+)?)"
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
            ("UART0" in (p.description or "").upper()) * 15,
        ),
        reverse=True,
    )
    if not ranked:
        raise RuntimeError("未找到 COM 口，请确认板子 USB 已连接")
    return ranked[0].device


def health_check(sample: dict[str, float]) -> list[str]:
    notes: list[str] = []
    az = sample["az"]
    gx = abs(sample["gx"])
    gy = abs(sample["gy"])
    gz = abs(sample["gz"])
    roll = abs(sample["roll"])
    pitch = abs(sample["pitch"])

    if abs(az - 9.81) > 2.5:
        notes.append(f"加速度 Z 异常 ({az:.2f} m/s²，平放应接近 9.81)")
    if math.hypot(sample["ax"], sample["ay"]) > 3.0 and abs(az) < 6.0:
        notes.append("加速度模长异常，检查安装方向或量程")
    if gx > 5.0 or gy > 5.0 or gz > 5.0:
        notes.append(f"陀螺仪偏大 (|g|={gx:.2f},{gy:.2f},{gz:.2f} °/s)，请保持静止或重校准")
    if roll > 20.0 or pitch > 20.0:
        notes.append(f"姿态角偏大 (roll={sample['roll']:.1f}° pitch={sample['pitch']:.1f}°)")
    if not notes:
        notes.append("姿态/IMU 读数正常（平放静止假设）")
    return notes


def format_sample(sample: dict[str, float], seq: int) -> str:
    return (
        f"#{seq:04d} "
        f"acc[m/s²] x={sample['ax']:7.2f} y={sample['ay']:7.2f} z={sample['az']:7.2f} | "
        f"gyro[°/s] x={sample['gx']:7.3f} y={sample['gy']:7.3f} z={sample['gz']:7.3f} | "
        f"att[°] R={sample['roll']:6.1f} P={sample['pitch']:6.1f} Y={sample['yaw']:6.1f} | "
        f"T={sample['temp']:4.1f}°C"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="PACER IMU 串口监视器")
    parser.add_argument("--port", help="COM 口，默认自动选 CH347 SERIAL-A")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率，默认 105600")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数，0=一直运行")
    parser.add_argument("--check-every", type=int, default=10, help="每 N 帧打印一次健康检查")
    args = parser.parse_args()

    port = pick_com_port(args.port)
    print(f"打开 {port} @ {args.baud} (DTR/RTS 关闭)")
    print("等待固件输出 PACER IMU ...  按 Ctrl+C 退出")
    print("-" * 100)

    seq = 0
    err_count = 0
    bias_printed = False
    debug_on = False
    end_time = time.time() + args.duration if args.duration > 0 else None

    with serial.Serial(port, args.baud, timeout=READ_TIMEOUT) as sp:
        sp.dtr = False
        sp.rts = False
        buf = ""

        while True:
            if end_time is not None and time.time() >= end_time:
                break

            chunk = sp.read(4096)
            if chunk:
                buf += chunk.decode("utf-8", errors="replace")

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if "PACER IMU READ ERR" in line:
                    err_count += 1
                    print(f"[ERR] I2C 读取失败 (累计 {err_count})")
                    continue

                if "PACER IMU DEBUG ON" in line:
                    debug_on = True
                    print("[INFO] IMU 调试流已开始")
                    continue

                if "PACER NO IMU" in line:
                    print("[FAIL] 固件未识别 IMU，请检查 B6/B7 接线")
                    return 2

                m_bias = BIAS_RE.search(line)
                if m_bias and not bias_printed:
                    bias_printed = True
                    print(
                        "[BIAS] "
                        f"gx={float(m_bias['gx']):.3f} "
                        f"gy={float(m_bias['gy']):.3f} "
                        f"gz={float(m_bias['gz']):.3f} °/s"
                    )
                    continue

                m = IMU_RE.search(line)
                if m:
                    sample = {k: float(m[k]) for k in m.groupdict()}
                    seq += 1
                    print(format_sample(sample, seq))
                    if seq % args.check_every == 0:
                        for note in health_check(sample):
                            print(f"       [CHECK] {note}")
                    continue

                if line.startswith("PACER "):
                    print(f"[BOOT] {line}")

            time.sleep(0.02)

    print("-" * 100)
    print(f"结束: 共收到 {seq} 帧 IMU 数据, I2C 错误 {err_count} 次")
    if seq == 0:
        print("未收到 IMU 数据。请确认已烧录带 CFG_IMU_DEBUG 的固件，且 BOOT0=0 已 RESET。")
        return 1
    if err_count > seq // 2:
        print("I2C 错误过多，姿态不可靠。")
        return 1
    print("可继续用手倾斜板子，观察 roll/pitch 是否跟随变化。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
