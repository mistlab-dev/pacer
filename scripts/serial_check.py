#!/usr/bin/env python3
"""自动扫描 CH347/COM 口并读取 PACER 串口输出。"""

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

BAUDS = (105600, 115200)
READ_SEC = 12
MARKERS = ("PACER", "PACER ALIVE", "PACER BOOT", "PACER INIT", "pacer>")


def score_port(name: str, desc: str) -> int:
    s = 0
    text = (name + " " + desc).upper()
    if "CH347" in text:
        s += 10
    if "SERIAL-A" in text or "SERIAL_A" in text:
        s += 20
    if "UART0" in text:
        s += 15
    return s


def try_read(port: str, baud: int) -> tuple[int, str]:
    try:
        with serial.Serial(port, baud, timeout=0.3) as sp:
            sp.dtr = False
            sp.rts = False
            time.sleep(0.2)
            end = time.time() + READ_SEC
            chunks = []
            while time.time() < end:
                n = sp.in_waiting
                if n:
                    chunks.append(sp.read(n))
                else:
                    time.sleep(0.1)
            data = b"".join(chunks)
            text = data.decode("utf-8", errors="replace")
            return len(data), text
    except Exception as e:
        return -1, str(e)


def main() -> int:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未找到任何 COM 口。请确认板子 USB 已插上。")
        return 1

    print("检测到的串口：")
    ranked = sorted(ports, key=lambda p: score_port(p.device, p.description), reverse=True)
    for p in ranked:
        print(f"  {p.device}: {p.description}")

    print()
    print(f"正在读取（{READ_SEC} 秒，尝试波特率 {BAUDS}）...")

    best = None
    for p in ranked:
        for baud in BAUDS:
            n, text = try_read(p.device, baud)
            if n < 0:
                print(f"  {p.device} @{baud}: 打开失败 — {text}")
                continue
            hit = any(m in text for m in MARKERS)
            label = f"  {p.device} @{baud}: {n} 字节"
            if hit:
                label += "  <-- PACER"
            print(label)
            if n > 0 and hit:
                print("-" * 40)
                print(text[:2000])
                print("-" * 40)
            if hit and (best is None or n > best[0]):
                best = (n, p.device, baud, text)

    print()
    if best:
        print(f"结论: PACER 输出在 {best[1]}，波特率 {best[2]}")
        return 0

    print("结论: 有 COM 口，但未收到 PACER 日志。")
    print("可能原因:")
    print("  1. 固件未烧录或 BOOT0 不在运行模式")
    print("  2. 板载 UART0 未接 USART1（需烧录最新固件）")
    print("  3. 请按一下板子 RESET 后再运行本脚本")
    return 2


if __name__ == "__main__":
    sys.exit(main())
