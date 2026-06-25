#!/usr/bin/env python3
"""
NDI Aurora / Polaris 等系统：USB 串口探测与 VER 数据交换测试。
协议与 Windows CombinedAPISample 中 CommandConstruction / SystemCRC 一致。

用法:
  python3 ndi_serial_probe.py
  python3 ndi_serial_probe.py -p /dev/ttyUSB1
  python3 ndi_serial_probe.py --no-break   # 不发送 BREAK，仅发 VER 4

需将用户加入 dialout 组: sudo usermod -aG dialout $USER 后重新登录。
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError as e:
    print("请先安装: pip3 install --user pyserial", file=sys.stderr)
    raise SystemExit(1) from e


def _crc16_table() -> list[int]:
    t: list[int] = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ (0xA001 if c & 1 else 0)
        t.append(c & 0xFFFF)
    return t

_CRC16 = _crc16_table()


def crc16(crc: int, b: int) -> int:
    return _CRC16[(crc ^ b) & 0xFF] ^ (crc >> 8)


def ndi_frame(cmd: str) -> bytes:
    """首空格改为 ':'，滚动 CRC-16，追加 4 位大写 hex + CR。"""
    buf = bytearray(cmd.encode("ascii"))
    for i in range(len(buf)):
        if buf[i] == 32:
            buf[i] = 58  # ':'
            break
    c = 0
    for b in buf:
        c = crc16(c, b)
    return bytes(buf) + f"{c:04X}".encode("ascii") + b"\r"


def reply_crc_ok(line: bytes) -> bool:
    """与 SystemCheckCRC 中文本分支一致：整行以 CR 结束，末 4 字符为 CRC hex。"""
    if not line.endswith(b"\r") or len(line) < 6:
        return False
    body = line[:-1]
    data, hx = body[:-4], body[-4:]
    try:
        want = int(hx, 16)
    except ValueError:
        return False
    c = 0
    for b in data:
        c = crc16(c, b)
    return (c & 0xFFFF) == want


def read_until_cr(ser: serial.Serial, total_timeout: float) -> bytes:
    deadline = time.monotonic() + total_timeout
    buf = b""
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if b"\r" in buf:
                i = buf.index(b"\r")
                return buf[: i + 1]
        time.sleep(0.02)
    return buf


def main() -> int:
    ap = argparse.ArgumentParser(description="NDI 串口 BREAK + VER 4 探测")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口设备路径")
    ap.add_argument(
        "-b", "--baud", type=int, default=9600, help="初始波特率（与示例打开端口一致）"
    )
    ap.add_argument("--no-break", action="store_true", help="不发送串口 BREAK")
    ap.add_argument(
        "--post-reset-wait",
        type=float,
        default=3.0,
        help="收到 RESET 应答后的等待秒数（与示例 nInitTO 一致，可改小加快调试）",
    )
    args = ap.parse_args()

    print("示例命令帧 VER 4:", ndi_frame("VER 4"))

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=2.0,
        )
    except serial.SerialException as e:
        print(f"无法打开 {args.port}: {e}", file=sys.stderr)
        print("若无权限: sudo usermod -aG dialout $USER 后重新登录", file=sys.stderr)
        return 1

    with ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if not args.no_break:
            ser.send_break(duration=0.25)
            time.sleep(0.5)
            r1 = read_until_cr(ser, 8.0)
            print("BREAK 后应答 (%d 字节):" % len(r1), r1[:300])
            if r1:
                print("  CRC 校验:", reply_crc_ok(r1))
            if args.post_reset_wait > 0:
                time.sleep(args.post_reset_wait)
        else:
            print("已跳过 BREAK")

        payload = ndi_frame("VER 4")
        ser.write(payload)
        ser.flush()
        r2 = read_until_cr(ser, 4.0)
        print("VER 4 应答 (%d 字节):" % len(r2), r2[:500])
        if r2:
            print("  CRC 校验:", reply_crc_ok(r2))

    print("完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
