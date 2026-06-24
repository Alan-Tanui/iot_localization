# Indoor BLE Localization System


---

## Project Structure

```
iot_localization/
│
├── firmware/
│   ├── anchor_node/anchor_node.ino   ← Flash to 3 ESP32s (change ANCHOR_ID each time)
│   └── tag_node/tag_node.ino         ← Flash to 1 ESP32 (the tracked device)
│
├── calibration/
│   ├── collect_rssi.py               ← Step 1: collect RSSI at known distances
│   └── fit_path_loss.py              ← Step 2: compute RSSI_0 and n
│
├── localization/
│   ├── trilateration.py              ← Core algorithm (RSSI → distance → position)
│   ├── kalman_filter.py              ← Smoothing filter (improves accuracy ~30-40%)
│   └── error_analysis.py            ← Generate CDF and heatmap graphs for report
│
├── gui/
│   └── live_map.py                   ← Real-time position display
│
├── requirements.txt
└── README.md
```

---

## Quick Start

### 1. Install Python dependencies
```bash
pip install -r requirements.txt
```

### 2. Flash firmware
- Open `firmware/anchor_node/anchor_node.ino` in Arduino IDE
- Set `ANCHOR_ID = 1`, flash to first ESP32
- Change to `ANCHOR_ID = 2`, flash to second ESP32
- Change to `ANCHOR_ID = 3`, flash to third ESP32
- Flash `firmware/tag_node/tag_node.ino` to the fourth ESP32 (the tag)

### 3. Calibrate the path loss model
Place the tag at known distances (1m, 2m, 3m, 4m, 5m) from one anchor.
Collect ~50 readings at each distance, then:
```bash
cd calibration
python collect_rssi.py --ports /dev/ttyUSB0 --output calibration_data.csv --duration 30
# Edit MEASURED dict in fit_path_loss.py with your averaged readings
python fit_path_loss.py
```

### 4. Run the live map
```bash
cd gui
# With real hardware:
python live_map.py --ports /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2

# Demo mode (no hardware needed):
python live_map.py --demo
```

### 5. Generate error analysis graphs (for report)
```bash
cd localization
python error_analysis.py
```

---

## Hardware Required

| Component         | Qty | Cost (approx.) |
|-------------------|-----|----------------|
| ESP32 dev board   | 4   | ~$15 total     |
| USB cables        | 4   | ~$8            |
| USB hub / laptop  | 1   | (you have)     |
| Measuring tape    | 1   | ~$2            |
| **Total**         |     | **~$25**       |

---

## Key Parameters (edit in trilateration.py after calibration)

| Parameter | Meaning                          | Typical value |
|-----------|----------------------------------|---------------|
| `RSSI_0`  | RSSI at 1m reference distance    | -40 to -50 dBm |
| `N`       | Path loss exponent               | 2.0–3.5 (indoor) |
| `ROOM_WIDTH` / `ROOM_HEIGHT` | Room dimensions in meters | measure yours |
| `ANCHOR_POSITIONS` | Known (x,y) of each anchor | measure yours |

---

## Serial Port Names by OS

| OS      | Port format              |
|---------|--------------------------|
| Linux   | `/dev/ttyUSB0`, `/dev/ttyUSB1` ... |
| macOS   | `/dev/cu.usbserial-...`  |
| Windows | `COM3`, `COM4`, `COM5` ...         |
