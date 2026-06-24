"""
live_map.py — Real-time indoor localization visualization.

Reads RSSI data from serial ports, runs trilateration + Kalman filter,
and displays a live map with the estimated tag position.

Usage:
    # Real hardware (3 ESP32s connected):
    python live_map.py --ports /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2

    # Demo mode (no hardware needed — uses simulated movement):
    python live_map.py --demo

Dependencies:
    pip install pyserial numpy scipy matplotlib
"""

import sys
import threading
import time
import json
import argparse
import math
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import deque
from pathlib import Path

# Add the project root directory to sys.path so we can import our modules
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(PROJECT_ROOT))

from localization.trilateration import (
    trilaterate, rssi_to_distance, AnchorReading,
    ANCHOR_POSITIONS, ROOM_WIDTH, ROOM_HEIGHT, RSSI_0, N
)
from localization.kalman_filter import KalmanFilter2D

# ---- Settings ----
SERIAL_BAUD     = 115200
UPDATE_INTERVAL = 200   # ms between plot refreshes
TRAIL_LENGTH    = 30    # number of past positions to show


class LocalizationApp:

    def __init__(self, ports=None, demo=False):
        self.demo     = demo
        self.ports    = ports or []
        self.kf       = KalmanFilter2D(process_noise=0.1, measurement_noise=1.5)
        self.stop     = threading.Event()

        # Shared state (written by serial threads, read by plot thread)
        self.rssi_latest   = {}   # {anchor_id: rssi_dBm}
        self.pos_raw       = None
        self.pos_filtered  = None
        self.trail_raw      = deque(maxlen=TRAIL_LENGTH)
        self.trail_filtered = deque(maxlen=TRAIL_LENGTH)
        self.errors        = []   # For error analysis
        self.lock          = threading.Lock()

        # Demo: simulated tag path
        self._demo_t = 0.0

    # ------------------------------------------------------------------ #
    #  Serial reader thread
    # ------------------------------------------------------------------ #
    def _serial_reader(self, port, anchor_id):
        import serial
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=2)
        except Exception as e:
            print(f"[Serial] Cannot open {port}: {e}")
            return

        while not self.stop.is_set():
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("{"):
                    continue
                data = json.loads(line)
                with self.lock:
                    self.rssi_latest[anchor_id] = data["rssi"]
            except Exception:
                continue
        ser.close()

    # ------------------------------------------------------------------ #
    #  Demo: synthetic RSSI from a moving tag
    # ------------------------------------------------------------------ #
    def _demo_tick(self):
        """Simulate a tag tracing a figure-8 path around the room."""
        t = self._demo_t
        cx, cy = ROOM_WIDTH / 2, ROOM_HEIGHT / 2
        rx, ry = ROOM_WIDTH * 0.35, ROOM_HEIGHT * 0.35
        # Lissajous figure-8
        tag_x = cx + rx * math.sin(t)
        tag_y = cy + ry * math.sin(2 * t)
        self._demo_t += 0.05

        rssi_dict = {}
        for aid, (ax, ay) in ANCHOR_POSITIONS.items():
            d = max(math.sqrt((tag_x - ax)**2 + (tag_y - ay)**2), 0.1)
            rssi_true  = RSSI_0 - 10 * N * math.log10(d)
            rssi_noisy = rssi_true + random.gauss(0, 3)
            rssi_dict[aid] = rssi_noisy

        with self.lock:
            self.rssi_latest = rssi_dict

        return tag_x, tag_y  # Ground truth (for error calculation in demo)

    # ------------------------------------------------------------------ #
    #  Localization update (called each animation frame)
    # ------------------------------------------------------------------ #
    def _update_position(self):
        with self.lock:
            rssi_snap = dict(self.rssi_latest)

        if len(rssi_snap) < 3:
            return

        readings = [AnchorReading(anchor_id=k, rssi=v) for k, v in rssi_snap.items()]
        raw = trilaterate(readings)
        if raw is None:
            return

        filtered = self.kf.update(*raw)

        with self.lock:
            self.pos_raw      = raw
            self.pos_filtered = filtered
            self.trail_raw.append(raw)
            self.trail_filtered.append(filtered)

    # ------------------------------------------------------------------ #
    #  Matplotlib setup
    # ------------------------------------------------------------------ #
    def _build_figure(self):
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle("Indoor BLE Localization System", fontsize=14, fontweight="bold")

        # --- Left: live map ---
        ax_map = axes[0]
        ax_map.set_xlim(-0.3, ROOM_WIDTH  + 0.3)
        ax_map.set_ylim(-0.3, ROOM_HEIGHT + 0.3)
        ax_map.set_aspect("equal")
        ax_map.set_xlabel("x (m)")
        ax_map.set_ylabel("y (m)")
        ax_map.set_title("Live Position Map")
        ax_map.grid(True, alpha=0.25)

        # Room outline
        room = patches.Rectangle((0, 0), ROOM_WIDTH, ROOM_HEIGHT,
                                  linewidth=2, edgecolor="navy", facecolor="#f0f4ff", alpha=0.4)
        ax_map.add_patch(room)

        # Anchors
        colors_anchor = ["royalblue", "seagreen", "tomato"]
        for (aid, (ax, ay)), col in zip(ANCHOR_POSITIONS.items(), colors_anchor):
            ax_map.plot(ax, ay, "^", markersize=14, color=col, zorder=5,
                        label=f"Anchor {aid} ({ax},{ay})")
            ax_map.annotate(f"A{aid}", (ax, ay), textcoords="offset points",
                            xytext=(6, 6), fontsize=9, color=col)

        ax_map.legend(loc="upper right", fontsize=8)

        # Dynamic artists
        trail_raw_line,      = ax_map.plot([], [], "o-", color="orange",
                                           markersize=3, linewidth=1, alpha=0.4, label="Raw trail")
        trail_filt_line,     = ax_map.plot([], [], "o-", color="purple",
                                           markersize=4, linewidth=1.5, alpha=0.7, label="Filtered trail")
        dot_raw  = ax_map.plot([], [], "o", color="orange",  markersize=10, zorder=8)[0]
        dot_filt = ax_map.plot([], [], "o", color="purple",  markersize=12, zorder=9)[0]
        txt_pos  = ax_map.text(0.02, 0.02, "", transform=ax_map.transAxes,
                               fontsize=9, color="purple",
                               bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        # RSSI bars (right panel)
        ax_rssi = axes[1]
        ax_rssi.set_title("RSSI per Anchor (dBm)")
        ax_rssi.set_ylim(-90, -30)
        ax_rssi.set_ylabel("RSSI (dBm)")
        ax_rssi.set_xticks([1, 2, 3])
        ax_rssi.set_xticklabels(["Anchor 1", "Anchor 2", "Anchor 3"])
        ax_rssi.grid(True, axis="y", alpha=0.3)
        bar_colors = ["royalblue", "seagreen", "tomato"]
        bars = ax_rssi.bar([1, 2, 3], [-60, -60, -60], color=bar_colors, alpha=0.7, width=0.5)
        txt_dist = ax_rssi.text(0.02, 0.96, "", transform=ax_rssi.transAxes,
                                fontsize=9, verticalalignment="top",
                                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        return fig, (trail_raw_line, trail_filt_line, dot_raw, dot_filt,
                     txt_pos, bars, txt_dist, ax_rssi)

    # ------------------------------------------------------------------ #
    #  Animation update callback
    # ------------------------------------------------------------------ #
    def _animate(self, frame, artists):
        (trail_raw_line, trail_filt_line, dot_raw, dot_filt,
         txt_pos, bars, txt_dist, ax_rssi) = artists

        # In demo mode, generate synthetic data
        gt = None
        if self.demo:
            gt = self._demo_tick()

        self._update_position()

        with self.lock:
            trail_r = list(self.trail_raw)
            trail_f = list(self.trail_filtered)
            pos_r   = self.pos_raw
            pos_f   = self.pos_filtered
            rssi_s  = dict(self.rssi_latest)

        # Update trails
        if len(trail_r) > 1:
            xs, ys = zip(*trail_r)
            trail_raw_line.set_data(xs, ys)
        if len(trail_f) > 1:
            xs, ys = zip(*trail_f)
            trail_filt_line.set_data(xs, ys)

        # Update position dots
        if pos_r:
            dot_raw.set_data([pos_r[0]], [pos_r[1]])
        if pos_f:
            dot_filt.set_data([pos_f[0]], [pos_f[1]])
            label = f"Pos: ({pos_f[0]:.2f}, {pos_f[1]:.2f}) m"
            if gt:
                err = math.sqrt((pos_f[0]-gt[0])**2 + (pos_f[1]-gt[1])**2)
                self.errors.append(err)
                label += f"\nError: {err:.2f} m"
            txt_pos.set_text(label)

        # Update RSSI bars
        for i, (aid, bar) in enumerate(zip(sorted(ANCHOR_POSITIONS.keys()), bars)):
            v = rssi_s.get(aid, -90)
            bar.set_height(v)

        # Distance info text
        dist_lines = []
        for aid in sorted(ANCHOR_POSITIONS.keys()):
            r = rssi_s.get(aid)
            if r is not None:
                d = rssi_to_distance(r)
                dist_lines.append(f"A{aid}: {r:.0f} dBm → {d:.2f} m")
        txt_dist.set_text("\n".join(dist_lines))
        ax_rssi.set_ylim(-90, -20)

        return (trail_raw_line, trail_filt_line, dot_raw, dot_filt,
                txt_pos, *bars, txt_dist)

    # ------------------------------------------------------------------ #
    #  Entry point
    # ------------------------------------------------------------------ #
    def run(self):
        if not self.demo:
            for i, port in enumerate(self.ports):
                t = threading.Thread(target=self._serial_reader,
                                     args=(port, i + 1), daemon=True)
                t.start()
            print(f"Reading from {len(self.ports)} serial ports...")
        else:
            print("Running in DEMO mode (simulated tag movement).")

        fig, artists = self._build_figure()
        anim = FuncAnimation(fig, self._animate, fargs=(artists,),
                             interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()

        if self.errors:
            avg = sum(self.errors) / len(self.errors)
            worst = max(self.errors)
            print(f"\nSession summary:")
            print(f"  Samples     : {len(self.errors)}")
            print(f"  Mean error  : {avg:.3f} m")
            print(f"  Max error   : {worst:.3f} m")


# ------------------------------------------------------------------ #
#  CLI
# ------------------------------------------------------------------ #
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Indoor BLE Localization Live Map")
    parser.add_argument("--ports", nargs="+",
                        help="Serial ports for 3 anchors e.g. /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2")
    parser.add_argument("--demo", action="store_true",
                        help="Run in demo mode with simulated movement (no hardware needed)")
    args = parser.parse_args()

    if not args.demo and not args.ports:
        print("Specify --ports or --demo. Running demo mode.\n")
        args.demo = True

    app = LocalizationApp(ports=args.ports, demo=args.demo)
    app.run()
