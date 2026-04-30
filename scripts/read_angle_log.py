import struct
import numpy as np
import matplotlib.pyplot as plt

STRUCT_FMT = "<I8f"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)  # 36 Bytes

samples = []
with open("log.bin", "rb") as f:
    while chunk := f.read(STRUCT_SIZE):
        if len(chunk) < STRUCT_SIZE:
            break
        samples.append(struct.unpack(STRUCT_FMT, chunk))

if not samples:
    print("Keine Daten.")
    exit()

data = np.array(samples)
ts            = data[:, 0] / 1000.0  # ms → s
gyro_x        = data[:, 1]
angle_raw     = data[:, 2]
angle_madgwick = data[:, 3]
angle_ekf     = data[:, 4]
angle_mahony  = data[:, 5]
acc_x         = data[:, 6]
acc_y         = data[:, 7]
acc_z         = data[:, 8]

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

axes[0].set_title("Winkel – Filtervergleich")
axes[0].plot(ts, angle_raw,      label="Raw (integriert)", alpha=0.6)
axes[0].plot(ts, angle_madgwick, label="Madgwick")
axes[0].plot(ts, angle_ekf,      label="EKF")
axes[0].plot(ts, angle_mahony,   label="Mahony")
axes[0].set_ylabel("Grad")
axes[0].legend()
axes[0].grid(True)

axes[1].set_title("Gyro X")
axes[1].plot(ts, gyro_x, color="gray")
axes[1].set_ylabel("rad/s")
axes[1].grid(True)

axes[2].set_title("Acceleration")
axes[2].plot(ts, acc_x, label="acc_x")
axes[2].plot(ts, acc_y, label="acc_y")
axes[2].plot(ts, acc_z, label="acc_z")
axes[2].set_ylabel("m/s²")
axes[2].set_xlabel("Zeit (s)")
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.savefig("log_plot.png", dpi=150)
plt.show()