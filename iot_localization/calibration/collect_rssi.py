"""
collect_rssi.py — Read RSSI data from multiple ESP32 anchor nodes over serial.

Usage:
    python collect_rssi.py --ports /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
                           --output rssi_data.csv
                           --duration 30

Dependencies:
    pip install pyserial
"""

import serial
import json
import threading
import csv
import time
import argparse
from datetime import datetime

# ---- Configuration ----
BAUD_RATE   = 115200
TIMEOUT_S   = 2

# Anchor physical positions in meters (measure these carefully in your room)
ANCHOR_POSITIONS = {
    1: (0.0,  0.0),   # Anchor 1: top-left corner
    2: (4.0,  0.0),   # Anchor 2: top-right corner
    3: (2.0,  3.0),   # Anchor 3: bottom-center
}

rows = []
lock = threading.Lock()
stop_event = threading.Event()


def read_anchor(port: str, anchor_id: int):
    """Thread function: reads JSON lines from one anchor over serial."""
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_S)
        print(f"[Anchor {anchor_id}] Connected on {port}")
    except serial.SerialException as e:
        print(f"[Anchor {anchor_id}] ERROR: {e}")
        return

    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line.startswith("{"):
                continue
            data = json.loads(line)
            ax, ay = ANCHOR_POSITIONS.get(anchor_id, (0, 0))
            row = {
                "timestamp":  datetime.now().isoformat(),
                "anchor_id":  anchor_id,
                "anchor_x":   ax,
                "anchor_y":   ay,
                "rssi":       data["rssi"],
            }
            with lock:
                rows.append(row)
            print(f"  Anchor {anchor_id} → RSSI {data['rssi']} dBm")
        except (json.JSONDecodeError, KeyError):
            continue
        except Exception as e:
            print(f"[Anchor {anchor_id}] Read error: {e}")
            break

    ser.close()


def main():
    parser = argparse.ArgumentParser(description="Collect RSSI from ESP32 anchors")
    parser.add_argument("--ports",    nargs="+", required=True,
                        help="Serial ports, one per anchor e.g. /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2")
    parser.add_argument("--output",   default="rssi_data.csv")
    parser.add_argument("--duration", type=int, default=60,
                        help="Collection duration in seconds")
    args = parser.parse_args()

    if len(args.ports) != len(ANCHOR_POSITIONS):
        print(f"WARNING: {len(args.ports)} ports given but {len(ANCHOR_POSITIONS)} anchors configured.")

    threads = []
    for i, port in enumerate(args.ports):
        anchor_id = i + 1
        t = threading.Thread(target=read_anchor, args=(port, anchor_id), daemon=True)
        threads.append(t)
        t.start()

    print(f"\nCollecting for {args.duration} seconds... (Ctrl+C to stop early)\n")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nStopped by user.")

    stop_event.set()
    for t in threads:
        t.join(timeout=3)

    # Write CSV
    if rows:
        fieldnames = ["timestamp", "anchor_id", "anchor_x", "anchor_y", "rssi"]
        with open(args.output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nSaved {len(rows)} readings to '{args.output}'")
    else:
        print("\nNo data collected — check serial port connections.")


if __name__ == "__main__":
    main()
