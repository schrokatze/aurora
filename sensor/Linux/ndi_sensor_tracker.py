#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Linux 下 NDI Aurora 电磁跟踪：串口打开、初始化、启用工具、TX 循环获取位姿。

协议与 CRC 与 aurora传感器/sensor_tracker.py 中 NDISensorTracker 一致；
设备节点使用 /dev/ttyUSB*、/dev/ttyACM* 等（参见同目录 NDI_SERIAL_LINUX.md）。

示例:
  python3 ndi_sensor_tracker.py -p /dev/ttyUSB0 --sensors 1,2
  python3 ndi_sensor_tracker.py --enable-all
  python3 ndi_sensor_tracker.py --skip-reset -p /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _import_tracker():
    linux_dir = Path(__file__).resolve().parent
    repo_root = linux_dir.parent
    aurora_dir = repo_root / "aurora传感器"
    if not aurora_dir.is_dir():
        print(f"未找到目录: {aurora_dir}", file=sys.stderr)
        raise SystemExit(1)
    sys.path.insert(0, str(aurora_dir))
    try:
        from sensor_tracker import NDISensorTracker  # type: ignore
    except ImportError as e:
        print("无法导入 sensor_tracker，请确认 aurora传感器/sensor_tracker.py 存在。", file=sys.stderr)
        raise SystemExit(1) from e
    return NDISensorTracker


def _parse_ports(s: str) -> list[int]:
    out: list[int] = []
    for part in s.replace(" ", "").split(","):
        if not part:
            continue
        out.append(int(part, 10))
    return out


def main() -> int:
    NDISensorTracker = _import_tracker()

    ap = argparse.ArgumentParser(description="Linux NDI Aurora：实时 TX 位姿获取（复用 aurora 脚本逻辑）")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口设备路径")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="波特率")
    ap.add_argument(
        "--sensors",
        metavar="LIST",
        help="要启用的物理端口号，逗号分隔，例如 1,2（与 Windows 脚本交互输入等价）",
    )
    ap.add_argument(
        "--enable-all",
        action="store_true",
        help="启用系统报告的需要启用的全部句柄（等价于 enable_all_ports）",
    )
    ap.add_argument("--debug", action="store_true", help="NDISensorTracker 调试输出")
    ap.add_argument(
        "--skip-reset",
        action="store_true",
        help="跳过串口 BREAK 硬件复位（设备已在上电/复位就绪时可略过）",
    )
    args = ap.parse_args()

    if args.enable_all and args.sensors:
        print("不能同时使用 --enable-all 与 --sensors", file=sys.stderr)
        return 2

    print("=" * 70)
    print("NDI 电磁传感器跟踪（Linux）— 各传感器位置与姿态")
    print("=" * 70)

    tracker = NDISensorTracker(port=args.port, baudrate=args.baud, debug=args.debug)

    try:
        if not tracker.connect():
            print("无法连接串口，请检查设备路径与 dialout 组权限。", file=sys.stderr)
            return 1

        if not args.skip_reset:
            print("执行硬件复位...", end=" ", flush=True)
            if not tracker.hardware_reset():
                print("警告: 硬件复位可能失败，继续尝试初始化...")
            else:
                print("完成")
        else:
            print("已跳过硬件复位（--skip-reset）")

        print("初始化系统...", end=" ", flush=True)
        if not tracker.initialize_system():
            print("失败")
            return 1
        print("完成")

        if args.enable_all:
            print("\n启用全部待启用端口...")
            tracker.enable_all_ports()
        elif args.sensors:
            ports = _parse_ports(args.sensors)
            if not ports:
                print("无效的 --sensors", file=sys.stderr)
                return 2
            print(f"\n启用传感器端口: {ports}")
            tracker.enable_sensors(ports)
        else:
            print("\n" + "=" * 70)
            print("请输入要启用的传感器编号（逗号分隔，例如: 1,2）:")
            try:
                user_input = input().strip()
            except (EOFError, KeyboardInterrupt):
                print("已取消")
                return 1
            if not user_input:
                print("未输入端口号")
                return 1
            port_numbers = [int(x.strip()) for x in user_input.split(",") if x.strip().isdigit()]
            if not port_numbers:
                print("未输入有效的端口号")
                return 1
            print(f"\n启用传感器: {port_numbers}")
            tracker.enable_sensors(port_numbers)

        print("开始跟踪...", end=" ", flush=True)
        if not tracker.start_tracking():
            print("失败")
            return 1
        print("完成")

        tracker.run_tracking_loop()

    except Exception as e:
        print(f"发生错误: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        try:
            tracker.stop_tracking()
        except Exception:
            pass
        tracker.disconnect()
        print("程序结束")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
