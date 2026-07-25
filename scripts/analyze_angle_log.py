#!/usr/bin/env python3
"""Create diagnostic metrics and plots from xiao_s3_angle_diag CSV."""

import argparse
import csv
import math
import sys
from pathlib import Path

try:
    import numpy as np
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("Fehler: numpy/matplotlib fehlen. Installieren mit: "
             "python3 -m pip install -r scripts/requirements-angle-tools.txt")


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        sys.exit(f"Fehler: {path} enthält keine Datenzeilen.")
    required = ("timestamp_us", "dt_s", "output_angle_deg", "stationary",
                "mag_norm", "accel_norm", "gyro_bias_x_dps",
                "gyro_bias_y_dps", "gyro_bias_z_dps")
    missing = [name for name in required if name not in rows[0]]
    if missing:
        sys.exit("Fehler: CSV-Spalten fehlen: " + ", ".join(missing))
    return rows


def col(rows, name):
    try:
        return np.asarray([float(row[name]) for row in rows])
    except (ValueError, KeyError) as exc:
        sys.exit(f"Fehler: Spalte {name} ist ungültig: {exc}")


def main():
    parser = argparse.ArgumentParser(description="Winkel-, Bias- und Timinglog auswerten.")
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("angle_report"))
    args = parser.parse_args()
    rows = load_csv(args.csv_file)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    t = col(rows, "timestamp_us")
    t = (t - t[0]) * 1e-6
    dt = col(rows, "dt_s")
    angle = col(rows, "output_angle_deg")
    stationary = col(rows, "stationary") > 0.5
    invalid_dt = col(rows, "invalid_dt_count")
    clips = col(rows, "gyro_clip_count")

    finite = np.isfinite(dt) & np.isfinite(angle)
    if not np.all(finite):
        sys.exit(f"Fehler: {np.count_nonzero(~finite)} NaN/Inf-Datenzeilen gefunden.")

    drift_deg_min = math.nan
    stationary_points = stationary & (t >= t[0] + min(60.0, max(0.0, t[-1] / 5)))
    if np.count_nonzero(stationary_points) >= 20:
        drift_deg_min = np.polyfit(t[stationary_points], angle[stationary_points], 1)[0] * 60

    metrics = [
        f"samples={len(rows)}",
        f"duration_s={t[-1]:.3f}",
        f"dt_min_ms={np.min(dt) * 1000:.4f}",
        f"dt_mean_ms={np.mean(dt) * 1000:.4f}",
        f"dt_p99_ms={np.percentile(dt, 99) * 1000:.4f}",
        f"dt_max_ms={np.max(dt) * 1000:.4f}",
        f"invalid_dt_count={int(invalid_dt[-1])}",
        f"gyro_clip_count={int(clips[-1])}",
        f"stationary_fraction={np.mean(stationary):.4f}",
        f"stationary_drift_deg_per_min={drift_deg_min:.6f}",
        f"angle_peak_to_peak_deg={np.ptp(angle):.6f}",
    ]
    report = "\n".join(metrics) + "\n"
    (args.output_dir / "metrics.txt").write_text(report, encoding="utf-8")
    print(report, end="")

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    axes[0].plot(t, angle, label="output angle")
    axes[0].set_ylabel("degree")
    axes[0].grid(True)
    axes[0].legend()
    for axis in range(3):
        axes[1].plot(t, col(rows, f"gyro_bias_{'xyz'[axis]}_dps"),
                     label=f"bias {'xyz'[axis]}")
    axes[1].set_ylabel("dps")
    axes[1].grid(True)
    axes[1].legend()
    axes[2].plot(t, dt * 1000, label="dt")
    axes[2].axhline(30, color="red", linestyle="--", label="reject limit")
    axes[2].set_ylabel("ms")
    axes[2].grid(True)
    axes[2].legend()
    axes[3].plot(t, col(rows, "accel_norm"), label="|accel|")
    axes[3].plot(t, col(rows, "mag_norm"), label="|mag|")
    axes[3].set_xlabel("time [s]")
    axes[3].grid(True)
    axes[3].legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "angle_diagnostics.png", dpi=150)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(8, 5))
    axis.hist(dt * 1000, bins=100)
    axis.set_xlabel("dt [ms]")
    axis.set_ylabel("count")
    axis.grid(True)
    fig.tight_layout()
    fig.savefig(args.output_dir / "dt_histogram.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
