#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从带物理 SROM 芯片的 NDI Aurora 工具读取 .rom 配置并保存到文件。

流程（Setup 模式，非 TSTART）：
  INIT → PHSR → PSRCH → PSEL → PPRD（循环 16 块 × 64B = 1024B）

示例:
  python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 -o tool_port1.rom
  python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --handle 01 --device-id A896 -o tool.rom
  python3 Linux/ndi_srom_dump.py -p /dev/ttyUSB0 --sensor 1 --init-port -o tool.rom
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
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


def main() -> int:
    NDISensorTracker = _import_tracker()

    ap = argparse.ArgumentParser(
        description="从物理 SROM 芯片导出 NDI 工具 .rom 文件（PSRCH/PSEL/PPRD）"
    )
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="串口设备路径")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="波特率")
    ap.add_argument("--sensor", type=int, metavar="N", help="物理端口号 1-4")
    ap.add_argument("--handle", type=lambda x: int(x, 16), metavar="HH", help="端口句柄（十六进制，如 01）")
    ap.add_argument(
        "-o",
        "--output",
        help="输出 .rom 路径（默认 sensor_<时间戳>.rom）",
    )
    ap.add_argument(
        "--device-id",
        metavar="ID",
        help="已知 SROM 设备 ID（16 位 hex，跳过 PSRCH 搜索）",
    )
    ap.add_argument(
        "--init-port",
        action="store_true",
        help="读取前先 PINIT（部分工具需要先初始化）",
    )
    ap.add_argument(
        "--user-area",
        action="store_true",
        help="读用户区（PURD）而非整片 SROM（PPRD）",
    )
    ap.add_argument("--size", type=int, default=1024, help="读取字节数，须为 64 的倍数（默认 1024）")
    ap.add_argument("--skip-reset", action="store_true", help="跳过 BREAK 硬件复位")
    ap.add_argument("--debug", action="store_true", help="调试输出")
    args = ap.parse_args()

    if not args.sensor and args.handle is None:
        print("请指定 --sensor 或 --handle", file=sys.stderr)
        return 2

    output = args.output
    if not output:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"port{args.sensor}" if args.sensor else f"handle{args.handle:02X}"
        output = f"srom_{tag}_{ts}.rom"

    print("=" * 70)
    print("NDI 物理 SROM 导出")
    print("=" * 70)
    print("说明: 物理 SROM 可通过 PPRD 读回；须在 Setup 模式（不要 TSTART）。")
    print("=" * 70)

    tracker = NDISensorTracker(port=args.port, baudrate=args.baud, debug=args.debug)

    try:
        if not tracker.connect():
            return 1

        if not args.skip_reset:
            print("硬件复位...", end=" ", flush=True)
            tracker.hardware_reset()
            print("完成")

        print("INIT...", end=" ", flush=True)
        if not tracker.initialize_system():
            print("失败")
            return 1
        print("完成")

        if args.handle is not None:
            handle = args.handle
            print(f"使用指定句柄: {handle:02X}")
        else:
            print(f"解析端口 {args.sensor} 的句柄...")
            handle = tracker.resolve_handle_for_port(args.sensor)
            if handle is None:
                print(f"未找到端口 {args.sensor} 的句柄，请确认传感器已插入并上电。", file=sys.stderr)
                return 1
            print(f"句柄: {handle:02X}")

        info = tracker.get_port_information(handle)
        if info:
            print(
                f"工具信息: type={info.get('tool_type')} "
                f"serial={info.get('serial_no')} "
                f"port={info.get('physical_port')}"
            )

        ok = tracker.dump_physical_srom(
            handle=handle,
            output_path=output,
            device_id=args.device_id,
            init_port=args.init_port,
            user_area=args.user_area,
            size=args.size,
        )
        return 0 if ok else 1

    except KeyboardInterrupt:
        print("\n已取消")
        return 1
    finally:
        tracker.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
