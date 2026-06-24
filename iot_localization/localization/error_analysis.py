"""
error_analysis.py — Generate error analysis graphs for the project report.

Run this after collecting real positioning data to produce:
  1. CDF of positioning error
  2. Raw vs Kalman-filtered error comparison
  3. Error heatmap across the room

Dependencies:
    pip install numpy scipy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import sys
sys.path.insert(0, "..")
from localization.trilateration import (
    trilaterate, AnchorReading, ANCHOR_POSITIONS,
    ROOM_WIDTH, ROOM_HEIGHT, RSSI_0, N
)
from localization.kalman_filter import KalmanFilter2D
import random

random.seed(42)
np.random.seed(42)


# ---- Simulate a test dataset (replace with real measurements) ----
def simulate_dataset(n_points=200, noise_std=3.0):
    """Generate synthetic ground-truth + noisy RSSI pairs."""
    gt_positions = []
    raw_estimates = []
    filt_estimates = []

    kf = KalmanFilter2D(process_noise=0.1, measurement_noise=1.5)

    for _ in range(n_points):
        # Random true position inside room
        true_x = random.uniform(0.3, ROOM_WIDTH - 0.3)
        true_y = random.uniform(0.3, ROOM_HEIGHT - 0.3)
        gt_positions.append((true_x, true_y))

        readings = []
        for aid, (ax, ay) in ANCHOR_POSITIONS.items():
            d = max(np.sqrt((true_x-ax)**2 + (true_y-ay)**2), 0.1)
            rssi_true  = RSSI_0 - 10 * N * np.log10(d)
            rssi_noisy = rssi_true + np.random.normal(0, noise_std)
            readings.append(AnchorReading(anchor_id=aid, rssi=rssi_noisy))

        raw = trilaterate(readings)
        filt = kf.update(*raw) if raw else (true_x, true_y)

        raw_estimates.append(raw or (true_x, true_y))
        filt_estimates.append(filt)

    return np.array(gt_positions), np.array(raw_estimates), np.array(filt_estimates)


def euclidean_errors(gt, estimates):
    return np.sqrt(np.sum((gt - estimates)**2, axis=1))


def plot_cdf(raw_errors, filt_errors):
    fig, ax = plt.subplots(figsize=(8, 5))

    for errors, label, color in [
        (raw_errors,  "Raw trilateration", "steelblue"),
        (filt_errors, "Kalman filtered",   "tomato"),
    ]:
        sorted_e = np.sort(errors)
        cdf = np.arange(1, len(sorted_e)+1) / len(sorted_e)
        ax.plot(sorted_e, cdf * 100, linewidth=2, label=label, color=color)

    # Annotate 80th percentile
    for errors, color in [(raw_errors, "steelblue"), (filt_errors, "tomato")]:
        p80 = np.percentile(errors, 80)
        ax.axvline(p80, color=color, linestyle="--", alpha=0.5)
        ax.text(p80 + 0.02, 15, f"80%ile\n{p80:.2f}m", color=color, fontsize=8)

    ax.set_xlabel("Positioning Error (m)", fontsize=12)
    ax.set_ylabel("Cumulative Distribution (%)", fontsize=12)
    ax.set_title("CDF of Positioning Error — Raw vs Kalman Filtered", fontsize=13)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, None)
    ax.set_ylim(0, 100)
    plt.tight_layout()
    plt.savefig("cdf_error.png", dpi=150)
    print("Saved: cdf_error.png")
    return fig


def plot_error_heatmap(gt, raw_est, filt_est):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    raw_errors  = euclidean_errors(gt, raw_est)
    filt_errors = euclidean_errors(gt, filt_est)

    for ax, errors, title in [
        (axes[0], raw_errors,  "Raw Trilateration Error (m)"),
        (axes[1], filt_errors, "Kalman Filtered Error (m)"),
    ]:
        sc = ax.scatter(gt[:, 0], gt[:, 1], c=errors, cmap="RdYlGn_r",
                        s=60, vmin=0, vmax=2.0, edgecolors="none", alpha=0.8)
        plt.colorbar(sc, ax=ax, label="Error (m)")

        # Draw room
        import matplotlib.patches as patches
        room = patches.Rectangle((0,0), ROOM_WIDTH, ROOM_HEIGHT,
                                  linewidth=2, edgecolor="navy", facecolor="none")
        ax.add_patch(room)

        # Draw anchors
        for aid, (axp, ayp) in ANCHOR_POSITIONS.items():
            ax.plot(axp, ayp, "^", markersize=12, color="navy", zorder=5)
            ax.annotate(f"A{aid}", (axp, ayp), xytext=(5, 5),
                        textcoords="offset points", fontsize=9, color="navy")

        ax.set_xlim(-0.3, ROOM_WIDTH + 0.3)
        ax.set_ylim(-0.3, ROOM_HEIGHT + 0.3)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(title)
        ax.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig("error_heatmap.png", dpi=150)
    print("Saved: error_heatmap.png")
    return fig


def print_summary(raw_errors, filt_errors):
    print("\n" + "="*52)
    print("  POSITIONING ERROR SUMMARY")
    print("="*52)
    print(f"{'Metric':<25} {'Raw':>10} {'Filtered':>10}")
    print("-"*52)
    metrics = [
        ("Mean error (m)",   np.mean),
        ("Median error (m)", np.median),
        ("Std dev (m)",      np.std),
        ("80th pct (m)",     lambda x: np.percentile(x, 80)),
        ("95th pct (m)",     lambda x: np.percentile(x, 95)),
        ("Max error (m)",    np.max),
    ]
    for name, fn in metrics:
        print(f"  {name:<23} {fn(raw_errors):>10.3f} {fn(filt_errors):>10.3f}")

    reduction = (1 - np.mean(filt_errors)/np.mean(raw_errors)) * 100
    print(f"\n  Kalman filter reduced mean error by {reduction:.1f}%")
    print("="*52)


if __name__ == "__main__":
    print("Generating simulated dataset (replace with real data)...")
    gt, raw_est, filt_est = simulate_dataset(n_points=300, noise_std=3.5)

    raw_errors  = euclidean_errors(gt, raw_est)
    filt_errors = euclidean_errors(gt, filt_est)

    print_summary(raw_errors, filt_errors)
    plot_cdf(raw_errors, filt_errors)
    plot_error_heatmap(gt, raw_est, filt_est)
    plt.show()
