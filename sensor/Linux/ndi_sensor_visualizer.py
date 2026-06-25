#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NDI Aurora 传感器实时可视化：复用 ndi_sensor_tracker 的初始化与 TX 采集逻辑，
在 matplotlib 窗口中实时显示各传感器的位置 (X/Y/Z) 与姿态 (四元数 + 欧拉角 + 坐标轴)。

依赖:
  pip install -r Linux/requirements-viz.txt

示例:
  python3 Linux/ndi_sensor_visualizer.py -p /dev/ttyUSB0 --sensors 1,2,3,4
  python3 Linux/ndi_sensor_visualizer.py -p /dev/ttyUSB0 --sensors 1,2,3,4 --skip-reset
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Tuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.gridspec import GridSpec
except ImportError as e:
    print("Install dependencies: pip install matplotlib numpy", file=sys.stderr)
    raise SystemExit(1) from e


def _import_tracker():
    linux_dir = Path(__file__).resolve().parent
    repo_root = linux_dir.parent
    aurora_dir = repo_root / "aurora传感器"
    if not aurora_dir.is_dir():
        print(f"Directory not found: {aurora_dir}", file=sys.stderr)
        raise SystemExit(1)
    sys.path.insert(0, str(aurora_dir))
    try:
        from sensor_tracker import NDISensorTracker  # type: ignore
    except ImportError as e:
        print("Cannot import sensor_tracker. Check aurora传感器/sensor_tracker.py exists.", file=sys.stderr)
        raise SystemExit(1) from e
    return NDISensorTracker


def _parse_ports(s: str) -> List[int]:
    out: List[int] = []
    for part in s.replace(" ", "").split(","):
        if part:
            out.append(int(part, 10))
    return out


def quat_to_rotation_matrix(q0: float, qx: float, qy: float, qz: float) -> np.ndarray:
    q = np.array([q0, qx, qy, qz], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    q0, qx, qy, qz = q / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * q0), 2 * (qx * qz + qy * q0)],
            [2 * (qx * qy + qz * q0), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * q0)],
            [2 * (qx * qz - qy * q0), 2 * (qy * qz + qx * q0), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def quat_to_euler_deg(q0: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    """四元数 -> 欧拉角 Roll/Pitch/Yaw（度）。"""
    q0, qx, qy, qz = q0, qx, qy, qz
    n = math.sqrt(q0 * q0 + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    q0, qx, qy, qz = q0 / n, qx / n, qy / n, qz / n

    sinr_cosp = 2.0 * (q0 * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (q0 * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (q0 * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


@dataclass
class SensorSample:
    valid: bool = False
    status: str = "UNKNOWN"
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    error: float = 0.0


@dataclass
class SensorState:
    port: int
    color: str
    label: str
    trail: Deque[Tuple[float, float, float]] = field(default_factory=deque)
    latest: SensorSample = field(default_factory=SensorSample)


SENSOR_COLORS = {1: "#e74c3c", 2: "#2ecc71", 3: "#3498db", 4: "#f39c12"}
AXIS_COLORS = ("#ff3333", "#33cc33", "#3366ff")
AXIS_NAMES = ("X", "Y", "Z")


def _build_port_handle_map(tracker, ports: List[int]) -> Dict[int, int]:
    """建立 物理端口 -> 句柄 的稳定映射。"""
    handle_order = tracker.get_port_handles(0)
    if not handle_order:
        handle_order = sorted(tracker.handle_info.keys())

    port_map: Dict[int, int] = {}
    for handle in handle_order:
        port = tracker._port_number_for_handle(handle, handle_order)
        if port in ports:
            port_map[port] = handle

    for idx, port in enumerate(sorted(ports)):
        if port not in port_map and idx < len(handle_order):
            port_map[port] = handle_order[idx]
    return port_map


def _init_tracker(args, NDISensorTracker):
    tracker = NDISensorTracker(port=args.port, baudrate=args.baud, debug=args.debug)
    if not tracker.connect():
        print("Cannot open serial port. Check device path and dialout group.", file=sys.stderr)
        raise SystemExit(1)

    if not args.skip_reset:
        print("Hardware reset...", end=" ", flush=True)
        if not tracker.hardware_reset():
            print("Warning: hardware reset may have failed, continuing...")
        else:
            print("done")
    else:
        print("Skipped hardware reset (--skip-reset)")

    print("Initializing system...", end=" ", flush=True)
    if not tracker.initialize_system():
        print("failed")
        tracker.disconnect()
        raise SystemExit(1)
    print("done")

    if args.enable_all:
        print("Enabling all pending ports...")
        tracker.enable_all_ports()
    elif args.sensors:
        ports = _parse_ports(args.sensors)
        if not ports:
            print("Invalid --sensors value", file=sys.stderr)
            tracker.disconnect()
            raise SystemExit(2)
        print(f"Enabling sensor ports: {ports}")
        tracker.enable_sensors(ports)
    else:
        print("Use --sensors to specify ports, e.g. --sensors 1,2,3,4", file=sys.stderr)
        tracker.disconnect()
        raise SystemExit(2)

    print("Starting tracking...", end=" ", flush=True)
    if not tracker.start_tracking():
        print("failed")
        tracker.disconnect()
        raise SystemExit(1)
    print("done")
    return tracker


def _samples_from_tracker(
    tracker, ports: List[int], port_handle_map: Dict[int, int]
) -> Dict[int, SensorSample]:
    transforms = tracker.get_tx_transforms()
    handle_to_port = {h: p for p, h in port_handle_map.items()}

    by_port: Dict[int, SensorSample] = {p: SensorSample(status="NO DATA") for p in ports}
    for handle, transform in transforms.items():
        port = handle_to_port.get(handle)
        if port is None:
            port = tracker._port_number_for_handle(handle, list(port_handle_map.values()))
        if port not in ports:
            continue

        pos = transform.get("translation", {})
        rot = transform.get("rotation", {})
        q0 = float(rot.get("q0", 1.0))
        qx = float(rot.get("qx", 0.0))
        qy = float(rot.get("qy", 0.0))
        qz = float(rot.get("qz", 0.0))
        by_port[port] = SensorSample(
            valid=bool(transform.get("valid", False)),
            status=str(transform.get("status", "UNKNOWN")),
            position=(float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))),
            quaternion=(q0, qx, qy, qz),
            euler_deg=quat_to_euler_deg(q0, qx, qy, qz),
            error=float(transform.get("error", 0.0)),
        )
    return by_port


class NdiSensorVisualizer:
    def __init__(
        self,
        tracker,
        ports: List[int],
        trail_length: int = 150,
        axis_scale: float = 0.12,
        update_ms: int = 50,
    ):
        self.tracker = tracker
        self.ports = sorted(ports)
        self.port_handle_map = _build_port_handle_map(tracker, self.ports)
        self.axis_scale = axis_scale
        self.update_ms = update_ms
        self.frame_count = 0
        self._axis_len = 30.0

        self.states: Dict[int, SensorState] = {}
        for port in self.ports:
            self.states[port] = SensorState(
                port=port,
                color=SENSOR_COLORS.get(port, "#95a5a6"),
                label=f"Sensor{port}",
                trail=deque(maxlen=max(1, trail_length)),
            )

        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title("NDI Aurora — Live Sensor Pose")
        gs = GridSpec(3, 2, height_ratios=[2.2, 2.2, 1.0], hspace=0.35, wspace=0.25)

        self.ax3d = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_xy = self.fig.add_subplot(gs[0, 1])
        self.ax_xz = self.fig.add_subplot(gs[1, 0])
        self.ax_yz = self.fig.add_subplot(gs[1, 1])

        n = len(self.ports)
        info_gs = gs[2, :].subgridspec(1, max(n, 1))
        self.ax_info: Dict[int, plt.Axes] = {}
        for i, port in enumerate(self.ports):
            ax = self.fig.add_subplot(info_gs[0, i])
            ax.set_axis_off()
            ax.set_facecolor("#f7f7f7")
            self.ax_info[port] = ax

        self._setup_axes()
        self._dynamic_artists: Dict[int, dict] = {p: {} for p in self.ports}
        self._info_text: Dict[int, plt.Text] = {}

        for port in self.ports:
            self._info_text[port] = self.ax_info[port].text(
                0.05,
                0.95,
                "",
                transform=self.ax_info[port].transAxes,
                va="top",
                ha="left",
                family="monospace",
                fontsize=11,
                color=SENSOR_COLORS.get(port, "#333"),
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white", edgecolor=SENSOR_COLORS.get(port, "#999"), alpha=0.95),
            )

    def _setup_axes(self) -> None:
        self.ax3d.set_title("3D Position + Orientation (R=X  G=Y  B=Z)")
        self.ax3d.set_xlabel("X (mm)")
        self.ax3d.set_ylabel("Y (mm)")
        self.ax3d.set_zlabel("Z (mm)")
        self.ax3d.view_init(elev=28, azim=-55)

        for ax, title, xlabel, ylabel in (
            (self.ax_xy, "XY Top View", "X (mm)", "Y (mm)"),
            (self.ax_xz, "XZ Side View", "X (mm)", "Z (mm)"),
            (self.ax_yz, "YZ Side View", "Y (mm)", "Z (mm)"),
        ):
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.35)

    def _clear_dynamic(self, port: int) -> None:
        arts = self._dynamic_artists.get(port, {})
        for key, obj in list(arts.items()):
            if obj is None:
                continue
            if key in ("quivers3d", "axis_tip_texts", "proj_annots"):
                for item in obj:
                    try:
                        item.remove()
                    except Exception:
                        pass
            elif key == "trails":
                for ln in obj.values():
                    try:
                        ln.remove()
                    except Exception:
                        pass
            else:
                try:
                    obj.remove()
                except Exception:
                    pass
        self._dynamic_artists[port] = {}

    def _draw_sensor(self, port: int, sample: SensorSample) -> None:
        state = self.states[port]
        color = state.color
        arts: dict = {}
        self._clear_dynamic(port)

        if not sample.valid:
            self._info_text[port].set_text(
                f"Sensor{port}\n\nStatus: {sample.status}\n\nWaiting for valid pose..."
            )
            return

        x, y, z = sample.position
        q0, qx, qy, qz = sample.quaternion
        roll, pitch, yaw = sample.euler_deg

        # --- 3D 散点 + 轨迹 ---
        if state.trail:
            trail = np.array(state.trail)
            arts["trails"] = {}
            arts["trails"]["3d"], = self.ax3d.plot(
                trail[:, 0], trail[:, 1], trail[:, 2], color=color, alpha=0.5, linewidth=1.8
            )
            arts["trails"]["xy"], = self.ax_xy.plot(trail[:, 0], trail[:, 1], color=color, alpha=0.5, linewidth=1.5)
            arts["trails"]["xz"], = self.ax_xz.plot(trail[:, 0], trail[:, 2], color=color, alpha=0.5, linewidth=1.5)
            arts["trails"]["yz"], = self.ax_yz.plot(trail[:, 1], trail[:, 2], color=color, alpha=0.5, linewidth=1.5)

        arts["scatter3d"] = self.ax3d.scatter([x], [y], [z], c=color, s=120, depthshade=True, edgecolors="black", linewidths=0.6)
        arts["scatter_xy"] = self.ax_xy.scatter([x], [y], c=color, s=90, edgecolors="black", linewidths=0.5, zorder=5)
        arts["scatter_xz"] = self.ax_xz.scatter([x], [z], c=color, s=90, edgecolors="black", linewidths=0.5, zorder=5)
        arts["scatter_yz"] = self.ax_yz.scatter([y], [z], c=color, s=90, edgecolors="black", linewidths=0.5, zorder=5)

        # --- 3D 姿态坐标轴 ---
        rot = quat_to_rotation_matrix(q0, qx, qy, qz)
        arts["quivers3d"] = []
        arts["axis_tip_texts"] = []
        for i, (ac, name) in enumerate(zip(AXIS_COLORS, AXIS_NAMES)):
            d = rot[:, i] * self._axis_len
            q = self.ax3d.quiver(
                x, y, z, d[0], d[1], d[2],
                color=ac, linewidth=2.0, arrow_length_ratio=0.2, alpha=0.95,
            )
            arts["quivers3d"].append(q)
            tip = np.array([x, y, z]) + d * 1.05
            arts["axis_tip_texts"].append(
                self.ax3d.text(tip[0], tip[1], tip[2], name, color=ac, fontsize=9, fontweight="bold")
            )

        # --- 3D 旁注：实时位置 + 姿态 ---
        label_str = (
            f"Sensor{port}\n"
            f"Pos X={x:.1f} Y={y:.1f} Z={z:.1f}\n"
            f"Ori R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}"
        )
        arts["label3d"] = self.ax3d.text(
            x, y, z + self._axis_len * 0.6,
            label_str,
            color=color,
            fontsize=9,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor=color, alpha=0.85),
        )

        # --- 2D 投影旁注 ---
        arts["proj_annots"] = []
        for ax, px, py, ox, oy in (
            (self.ax_xy, x, y, 8, 8),
            (self.ax_xz, x, z, 8, 8),
            (self.ax_yz, y, z, 8, 8),
        ):
            ann = ax.annotate(
                f"Sensor{port} ({px:.0f},{py:.0f})",
                (px, py),
                textcoords="offset points",
                xytext=(ox, oy),
                fontsize=8,
                color=color,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor=color, alpha=0.8),
            )
            arts["proj_annots"].append(ann)

        # --- 底部信息面板：完整三坐标位置 + 三角度姿态 + 四元数 ---
        self._info_text[port].set_text(
            f"Sensor{port}\n"
            f"-----------------\n"
            f"Position (mm)\n"
            f"  X = {x:9.3f}\n"
            f"  Y = {y:9.3f}\n"
            f"  Z = {z:9.3f}\n"
            f"-----------------\n"
            f"Orientation (deg)\n"
            f"  Roll  = {roll:8.2f}\n"
            f"  Pitch = {pitch:8.2f}\n"
            f"  Yaw   = {yaw:8.2f}\n"
            f"-----------------\n"
            f"Quaternion\n"
            f"  q0 = {q0:7.4f}\n"
            f"  qx = {qx:7.4f}\n"
            f"  qy = {qy:7.4f}\n"
            f"  qz = {qz:7.4f}\n"
            f"Error = {sample.error:.4f}"
        )

        self._dynamic_artists[port] = arts

    def _update_limits(self, points: List[Tuple[float, float, float]]) -> None:
        if not points:
            return
        arr = np.array(points)
        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)
        center = (mins + maxs) / 2.0
        span = np.maximum(maxs - mins, 100.0)
        half = span / 2.0 + 30.0
        self._axis_len = max(20.0, float(np.max(span)) * self.axis_scale)

        self.ax3d.set_xlim(center[0] - half[0], center[0] + half[0])
        self.ax3d.set_ylim(center[1] - half[1], center[1] + half[1])
        self.ax3d.set_zlim(center[2] - half[2], center[2] + half[2])
        self.ax_xy.set_xlim(center[0] - half[0], center[0] + half[0])
        self.ax_xy.set_ylim(center[1] - half[1], center[1] + half[1])
        self.ax_xz.set_xlim(center[0] - half[0], center[0] + half[0])
        self.ax_xz.set_ylim(center[2] - half[2], center[2] + half[2])
        self.ax_yz.set_xlim(center[1] - half[1], center[1] + half[1])
        self.ax_yz.set_ylim(center[2] - half[2], center[2] + half[2])

    def poll_and_draw(self, _frame: int) -> None:
        samples = _samples_from_tracker(self.tracker, self.ports, self.port_handle_map)
        self.frame_count += 1
        valid_points: List[Tuple[float, float, float]] = []

        for port in self.ports:
            sample = samples[port]
            state = self.states[port]
            state.latest = sample
            if sample.valid:
                state.trail.append(sample.position)
                valid_points.append(sample.position)
            self._draw_sensor(port, sample)

        if valid_points:
            self._update_limits(valid_points)

        self.fig.suptitle(
            f"NDI Aurora Live Pose  |  Frame {self.frame_count}  |  Close window to exit",
            fontsize=12,
            y=0.98,
        )
        self.fig.canvas.draw_idle()

    def run(self) -> None:
        print("Opening visualization window (3D + projections + live readout)...")
        anim = FuncAnimation(
            self.fig,
            self.poll_and_draw,
            interval=self.update_ms,
            blit=False,
            cache_frame_data=False,
        )
        try:
            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plt.show()
        finally:
            anim.event_source.stop()


def main() -> int:
    NDISensorTracker = _import_tracker()

    ap = argparse.ArgumentParser(description="NDI Aurora live sensor visualization (matplotlib)")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Serial device path")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate")
    ap.add_argument("--sensors", metavar="LIST", help="Ports to enable, comma-separated, e.g. 1,2,3,4")
    ap.add_argument("--enable-all", action="store_true", help="Enable all pending ports")
    ap.add_argument("--debug", action="store_true", help="NDISensorTracker debug output")
    ap.add_argument("--skip-reset", action="store_true", help="Skip BREAK hardware reset")
    ap.add_argument("--trail", type=int, default=150, help="Trail length in points (default 150)")
    ap.add_argument("--axis-scale", type=float, default=0.12, help="Orientation axis length as fraction of view (default 0.12)")
    ap.add_argument("--interval", type=int, default=50, help="Refresh interval in ms (default 50)")
    args = ap.parse_args()

    if args.enable_all and args.sensors:
        print("Cannot use --enable-all and --sensors together", file=sys.stderr)
        return 2

    ports = _parse_ports(args.sensors) if args.sensors else list(range(1, 5))
    if args.enable_all:
        ports = list(range(1, 5))

    print("=" * 70)
    print("NDI Aurora Sensor Visualization")
    print("=" * 70)

    tracker = None
    try:
        tracker = _init_tracker(args, NDISensorTracker)
        visualizer = NdiSensorVisualizer(
            tracker,
            ports=ports,
            trail_length=args.trail,
            axis_scale=args.axis_scale,
            update_ms=args.interval,
        )
        visualizer.run()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        if tracker is not None:
            try:
                tracker.stop_tracking()
            except Exception:
                pass
            tracker.disconnect()
            print("Done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
