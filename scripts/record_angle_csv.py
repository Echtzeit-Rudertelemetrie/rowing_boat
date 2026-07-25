#!/usr/bin/env python3
"""Record the xiao_s3_angle_diag CSV stream without modifying data rows."""

import argparse
import csv
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("Fehler: pyserial fehlt. Installieren mit: "
             "python3 -m pip install -r scripts/requirements-angle-tools.txt")


REQUIRED_COLUMNS = {"timestamp_us", "dt_s", "gyro_raw_x_dps", "output_angle_deg"}
CSV_COLUMNS = [
    "timestamp_us", "sequence_number", "dt_s",
    "gyro_raw_x_dps", "gyro_raw_y_dps", "gyro_raw_z_dps",
    "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps",
    "gyro_corrected_x_dps", "gyro_corrected_y_dps", "gyro_corrected_z_dps",
    "accel_x", "accel_y", "accel_z", "accel_norm",
    "mag_raw_x", "mag_raw_y", "mag_raw_z",
    "mag_calibrated_x", "mag_calibrated_y", "mag_calibrated_z", "mag_norm",
    "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
    "yaw_deg", "pitch_deg", "roll_deg", "output_angle_deg",
    "accel_valid", "mag_valid", "mag_fresh", "stationary",
    "queue_depth", "dropped_events", "calibration_state", "temperature_c",
    "invalid_dt_count", "gyro_clip_count",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="CSV-Diagnosedaten des XIAO ESP32S3 aufzeichnen.")
    parser.add_argument("--port", required=True,
                        help="Serieller Port, z.B. /dev/ttyACM0 oder /dev/serial/by-id/...")
    parser.add_argument("--output", required=True, type=Path,
                        help="Ziel-CSV-Datei")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Aufzeichnungsdauer in Sekunden; 0 = bis Strg+C")
    return parser.parse_args()


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = 0
    header = None
    started = time.monotonic()

    try:
        port = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        sys.exit(f"Fehler: Port {args.port} kann nicht geöffnet werden: {exc}")

    print(f"Aufzeichnung nach {args.output}; Abbruch mit Strg+C", file=sys.stderr)
    try:
        with args.output.open("w", newline="", encoding="utf-8") as output:
            while args.duration <= 0 or time.monotonic() - started < args.duration:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#") or line.startswith("ANGLE_"):
                    print(line, file=sys.stderr)
                    continue
                fields = next(csv.reader([line]))
                if header is None:
                    candidate = set(fields)
                    if REQUIRED_COLUMNS.issubset(candidate):
                        header = fields
                        output.write(line + "\n")
                        output.flush()
                        continue
                    # The firmware prints its header only once after boot. If the
                    # recorder connects later, the first received line is already
                    # numeric data. Its fixed field count identifies the format.
                    if len(fields) == len(CSV_COLUMNS):
                        try:
                            [float(value) for value in fields]
                        except ValueError:
                            print(f"Übersprungen vor CSV-Header: {line}", file=sys.stderr)
                            continue
                        header = CSV_COLUMNS
                        output.write(",".join(header) + "\n")
                        output.write(line + "\n")
                        rows += 1
                        output.flush()
                        print("CSV-Header lokal ergänzt.", file=sys.stderr)
                        continue
                    print(f"Übersprungen vor CSV-Header: {line}", file=sys.stderr)
                    continue
                if len(fields) != len(header):
                    print(f"Warnung: unvollständige Zeile ({len(fields)}/{len(header)})",
                          file=sys.stderr)
                    continue
                output.write(line + "\n")
                rows += 1
                if rows % 100 == 0:
                    output.flush()
                    print(f"\r{rows} Zeilen", end="", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
    print(f"\nFertig: {rows} Datenzeilen in {args.output}", file=sys.stderr)
    if not header or rows == 0:
        sys.exit("Fehler: keine gültigen CSV-Daten empfangen.")


if __name__ == "__main__":
    main()
