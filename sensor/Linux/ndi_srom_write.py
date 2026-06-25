#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 .rom 文件烧录到空白/相同 NDI Aurora 工具的物理 SROM 芯片。

流程（Setup 模式，非 TSTART）：
  INIT → PHSR → PSRCH → PSEL → PPWR（循环 16 块 × 64B）→ PPRD 回读校验

示例:
  # 先查看空白硬件上的 SROM 设备 ID
  python3 Linux/ndi_srom_write.py -p /dev/ttyUSB0 --sensor 1 --list-devices

  # 将已导出的 rom 烧录到空白硬件（会要求确认）
  python3 Linux/ndi_srom_write.py -p /dev/ttyUSB0 --sensor 1 -i srom_port1.rom --confirm

  # 指定空白芯片的 SROM 设备 ID（PSRCH 搜到多个或 ID 特殊时）
  python3 Linux/ndi_srom_write.py -p /dev/ttyUSB0 --sensor 1 -i srom_port1.rom \\
      --device-id 0B1182B602000045 --confirm
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _import_tracker():
    linux_dir = Path(__file__).resolve().parent
    aurora_dir = linux_dir.parent / "aurora传感器"
    if not aurora_dir.is_dir():
        print(f"未找到目录: {aurora_dir}", file=sys.stderr)
        raise SystemExit(1)
    sys.path.insert(0, str(aurora_dir))
    try:
        from sensor_tracker import NDISensorTracker  # type: ignore
    except ImportError as e:
        print("无法导入 sensor_tracker。", file=sys.stderr)
        raise SystemExit(1) from e
    return NDISensorTracker


def _connect_and_init(tracker, skip_reset: bool) -> bool:
    if not tracker.connect():
        return False
    if not skip_reset:
        print("硬件复位...", end=" ", flush=True)
        tracker.hardware_reset()
        print("完成")
    print("INIT...", end=" ", flush=True)
    if not tracker.initialize_system():
        print("失败")
        return False
    print("完成")
    return True


def _resolve_handle(tracker, sensor: int | None, handle: int | None) -> int | None:
    if handle is not None:
        print(f"使用指定句柄: {handle:02X}")
        return handle
    print(f"解析端口 {sensor} 的句柄...")
    h = tracker.resolve_handle_for_port(sensor)
    if h is None:
        print(f"未找到端口 {sensor} 的句柄，请确认空白硬件已插入并上电。", file=sys.stderr)
        return None
    print(f"句柄: {h:02X}")
    return h


def main() -> int:
    NDISensorTracker = _import_tracker()

    ap = argparse.ArgumentParser(
        description="将 .rom 烧录到 NDI Aurora 物理 SROM 芯片（PSRCH/PSEL/PPWR）"
    )
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口设备路径")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="波特率")
    ap.add_argument("--sensor", type=int, metavar="N", help="物理端口号 1-4")
    ap.add_argument("--handle", type=lambda x: int(x, 16), metavar="HH", help="端口句柄（十六进制）")
    ap.add_argument("-i", "--input", help=".rom 文件路径（烧录时必填）")
    ap.add_argument(
        "--device-id",
        metavar="ID",
        help="SROM 设备 ID（16 位 hex，来自 PSRCH；空白芯片请先 --list-devices）",
    )
    ap.add_argument(
        "--list-devices",
        action="store_true",
        help="仅搜索并列出当前工具上的 SROM 设备 ID，不写入",
    )
    ap.add_argument(
        "--no-verify",
        action="store_true",
        help="写入后不做 PPRD 回读校验",
    )
    ap.add_argument(
        "--confirm",
        action="store_true",
        help="确认执行写入（未加此参数时仅预览，不会烧录）",
    )
    ap.add_argument("--skip-reset", action="store_true", help="跳过 BREAK 硬件复位")
    ap.add_argument("--debug", action="store_true", help="调试输出")
    args = ap.parse_args()

    if not args.sensor and args.handle is None:
        print("请指定 --sensor 或 --handle", file=sys.stderr)
        return 2

    if not args.list_devices and not args.input:
        print("烧录模式需要 -i/--input 指定 .rom 文件", file=sys.stderr)
        return 2

    tracker = NDISensorTracker(port=args.port, baudrate=args.baud, debug=args.debug)

    try:
        if not _connect_and_init(tracker, args.skip_reset):
            return 1

        handle = _resolve_handle(tracker, args.sensor, args.handle)
        if handle is None:
            return 1

        if args.list_devices:
            tracker.get_port_handles(2)
            ids = tracker.search_srom_devices(handle)
            if ids:
                print("检测到的 SROM 设备 ID：")
                for dev_id in ids:
                    print(f"  {dev_id}")
            else:
                print("未检测到 SROM 设备。")
            return 0

        rom_path = Path(args.input)
        if not rom_path.is_file():
            print(f"ROM 文件不存在: {rom_path}", file=sys.stderr)
            return 1

        print("=" * 70)
        print("即将烧录物理 SROM")
        print("=" * 70)
        print(f"  源文件 : {rom_path.resolve()}")
        print(f"  目标   : 端口 {args.sensor or '?'} / 句柄 {handle:02X}")
        print(f"  设备ID : {args.device_id or '（自动 PSRCH 搜索）'}")
        print()
        print("原理: PPWR 按 64 字节块写入工具线缆上的 SROM 芯片；")
        print("      与 PPRD 读取互为逆操作。须在 Setup 模式执行。")
        print()
        print("警告: 写入会覆盖芯片内原有配置，不可轻易撤销。")
        print("=" * 70)

        if not args.confirm:
            print("未加 --confirm，已取消写入。确认后请重新运行并加上 --confirm。")
            return 0

        ok = tracker.program_physical_srom(
            handle=handle,
            input_path=str(rom_path),
            device_id=args.device_id,
            verify=not args.no_verify,
        )
        if ok:
            print()
            print("烧录完成。建议后续验证：")
            print(f"  1. 重新导出: python3 Linux/ndi_srom_dump.py -p {args.port} --sensor {args.sensor or 1} -o verify.rom")
            print(f"  2. 对比文件: sha256sum {rom_path.name} verify.rom")
            print(f"  3. 跟踪测试: python3 Linux/ndi_sensor_tracker.py -p {args.port} --sensors {args.sensor or 1}")
        return 0 if ok else 1

    except KeyboardInterrupt:
        print("\n已取消")
        return 1
    finally:
        tracker.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
