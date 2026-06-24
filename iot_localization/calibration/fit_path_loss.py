"""
fit_path_loss.py — Calibrate the log-distance path loss model.

Procedure:
  1. Place one anchor node at position (0,0).
  2. Hold the tag at exactly 1m, 2m, 3m, 4m, 5m from the anchor.
  3. At each distance collect ~50 RSSI samples (run collect_rssi.py).
  4. Enter your averaged readings below and run this script.
  5. The script outputs RSSI_0 and n — paste them into trilateration.py.

Dependencies:
    pip install numpy scipy matplotlib
"""

import numpy as np
from scipy.stats import linregress
import matplotlib.pyplot as plt
import json, os

# ---------------------------------------------------------------
# STEP 1: Enter your measured averages here (dBm)
# ---------------------------------------------------------------
MEASURED = {
    # distance_m : avg_rssi_dBm
    1: -42,
    2: -51,
    3: -57,
    4: -62,
    5: -65,
}
# ---------------------------------------------------------------

def fit(measured: dict):
    distances = np.array(sorted(measured.keys()), dtype=float)
    rssi_vals = np.array([measured[d] for d in distances], dtype=float)
    log_d     = np.log10(distances)

    slope, intercept, r_value, p_value, std_err = linregress(log_d, rssi_vals)

    n      = -slope / 10.0
    rssi_0 = intercept          # RSSI at d=1m (since log10(1)=0)

    print("=" * 50)
    print("  PATH LOSS MODEL CALIBRATION RESULTS")
    print("=" * 50)
    print(f"  RSSI₀ (at 1 m)         : {rssi_0:.2f} dBm")
    print(f"  Path loss exponent (n) : {n:.3f}")
    print(f"  R² (fit quality)       : {r_value**2:.4f}")
    print(f"  Std error              : {std_err:.4f}")
    print("=" * 50)
    print("\n  Copy these into trilateration.py:")
    print(f"    RSSI_0 = {rssi_0:.2f}")
    print(f"    N      = {n:.3f}")

    # Save to JSON for use by other scripts
    params = {"rssi_0": rssi_0, "n": n, "r_squared": r_value**2}
    with open("path_loss_params.json", "w") as f:
        json.dump(params, f, indent=2)
    print("\n  Saved to path_loss_params.json")

    return n, rssi_0, distances, rssi_vals, slope, intercept

def plot(n, rssi_0, distances, rssi_vals, slope, intercept):
    d_range = np.linspace(0.5, 8, 200)
    rssi_fit = rssi_0 - 10 * n * np.log10(d_range)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Plot 1: RSSI vs distance
    ax1 = axes[0]
    ax1.scatter(distances, rssi_vals, color="steelblue", s=80, zorder=5, label="Measured averages")
    ax1.plot(d_range, rssi_fit, color="tomato", linewidth=2, label=f"Fitted: n={n:.2f}")
    ax1.set_xlabel("Distance (m)")
    ax1.set_ylabel("RSSI (dBm)")
    ax1.set_title("RSSI vs Distance")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: RSSI vs log10(d) — should be linear
    ax2 = axes[1]
    log_d = np.log10(distances)
    log_range = np.log10(d_range)
    ax2.scatter(log_d, rssi_vals, color="steelblue", s=80, zorder=5, label="Measured")
    ax2.plot(log_range, slope * log_range + intercept, color="tomato", linewidth=2,
             label=f"Linear fit (R²={intercept:.0f})")
    ax2.set_xlabel("log₁₀(distance)")
    ax2.set_ylabel("RSSI (dBm)")
    ax2.set_title("RSSI vs log₁₀(d)  [should be linear]")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("path_loss_calibration.png", dpi=150)
    print("  Plot saved: path_loss_calibration.png")
    plt.show()

if __name__ == "__main__":
    n, rssi_0, distances, rssi_vals, slope, intercept = fit(MEASURED)
    plot(n, rssi_0, distances, rssi_vals, slope, intercept)
