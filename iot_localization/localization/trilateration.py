"""
trilateration.py — Core indoor localization algorithm.

Converts RSSI readings from three anchors into an (x, y) position estimate
using the log-distance path loss model + weighted least-squares trilateration.

Dependencies:
    pip install numpy scipy
"""

import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json, os


# ---------------------------------------------------------------
# Path Loss Parameters — update after running fit_path_loss.py
# ---------------------------------------------------------------
RSSI_0 = -42.0   # RSSI at 1 meter reference distance (dBm)
N      = 2.8     # Path loss exponent (measured in your environment)
# ---------------------------------------------------------------

# Room boundaries (meters) — used for bounding the optimizer
ROOM_WIDTH  = 5.0
ROOM_HEIGHT = 4.0

# Known anchor positions (x_m, y_m)
ANCHOR_POSITIONS = {
    1: (0.0,        0.0),
    2: (ROOM_WIDTH, 0.0),
    3: (ROOM_WIDTH / 2, ROOM_HEIGHT),
}


@dataclass
class AnchorReading:
    anchor_id: int
    rssi: float  # dBm


def load_path_loss_params(path: str = "calibration/path_loss_params.json"):
    """Load calibrated parameters from JSON file if available."""
    global RSSI_0, N
    if os.path.exists(path):
        with open(path) as f:
            params = json.load(f)
        RSSI_0 = params["rssi_0"]
        N      = params["n"]
        print(f"[Trilateration] Loaded params: RSSI_0={RSSI_0:.2f}, N={N:.3f}")
    else:
        print(f"[Trilateration] Using default params (run fit_path_loss.py to calibrate)")


def rssi_to_distance(rssi: float, rssi_0: float = RSSI_0, n: float = N) -> float:
    """
    Convert RSSI (dBm) to distance (meters) using log-distance path loss model.

    RSSI(d) = RSSI_0 - 10 * n * log10(d / d0)    [d0 = 1m]
    → d = 10 ^ ((RSSI_0 - RSSI) / (10 * n))
    """
    if rssi >= rssi_0:
        return 0.1  # Tag is very close; clamp to avoid log(0)
    return 10 ** ((rssi_0 - rssi) / (10.0 * n))


def trilaterate(readings: List[AnchorReading]) -> Optional[Tuple[float, float]]:
    """
    Estimate tag position from 3+ anchor RSSI readings.

    Uses weighted least squares: anchors with stronger signal get higher weight
    (they are closer and their distance estimate is more reliable).

    Returns (x, y) in meters, or None if insufficient readings.
    """
    if len(readings) < 3:
        print("[Trilateration] Need at least 3 anchor readings.")
        return None

    anchors   = []
    distances = []
    weights   = []

    for r in readings:
        pos = ANCHOR_POSITIONS.get(r.anchor_id)
        if pos is None:
            continue
        d = rssi_to_distance(r.rssi)
        w = 1.0 / (d ** 2 + 0.01)  # Inverse-square weighting; +0.01 avoids div-by-zero

        anchors.append(pos)
        distances.append(d)
        weights.append(w)

    anchors   = np.array(anchors)
    distances = np.array(distances)
    weights   = np.array(weights)

    def cost(pos):
        """Weighted sum of squared residuals between estimated and measured distances."""
        estimated = np.sqrt((pos[0] - anchors[:, 0])**2 + (pos[1] - anchors[:, 1])**2)
        residuals = estimated - distances
        return np.sum(weights * residuals**2)

    # Initial guess: centroid of anchors
    x0 = np.mean(anchors, axis=0)

    # Bounded optimization — keep estimate inside the room
    bounds = [(0, ROOM_WIDTH), (0, ROOM_HEIGHT)]
    result = minimize(cost, x0, method="L-BFGS-B", bounds=bounds)

    if result.success or result.fun < 1.0:
        return float(result.x[0]), float(result.x[1])
    else:
        print(f"[Trilateration] Optimizer warning: {result.message}")
        return float(result.x[0]), float(result.x[1])  # Return best guess anyway


def compute_error(estimated: Tuple[float, float],
                  ground_truth: Tuple[float, float]) -> float:
    """Euclidean error in meters."""
    return np.sqrt((estimated[0] - ground_truth[0])**2 +
                   (estimated[1] - ground_truth[1])**2)


# ---- Quick self-test ----
if __name__ == "__main__":
    # Simulate tag at (2.0, 1.5)
    TRUE_POS = (2.0, 1.5)

    # Add Gaussian noise to simulate real RSSI
    np.random.seed(42)
    simulated_readings = []
    for aid, (ax, ay) in ANCHOR_POSITIONS.items():
        d_true  = np.sqrt((TRUE_POS[0]-ax)**2 + (TRUE_POS[1]-ay)**2)
        rssi_true = RSSI_0 - 10 * N * np.log10(max(d_true, 0.1))
        rssi_noisy = rssi_true + np.random.normal(0, 3)  # ±3 dBm noise
        simulated_readings.append(AnchorReading(anchor_id=aid, rssi=rssi_noisy))
        print(f"  Anchor {aid}: true d={d_true:.2f}m, RSSI={rssi_noisy:.1f} dBm")

    est = trilaterate(simulated_readings)
    err = compute_error(est, TRUE_POS)
    print(f"\nTrue position  : {TRUE_POS}")
    print(f"Estimated pos  : ({est[0]:.3f}, {est[1]:.3f})")
    print(f"Error          : {err:.3f} m")
