# MATLAB – Rowing Sensor Visualization

Live-Visualisierung und Algorithmen-Vergleich für ESP32-basiertes Rudersensor-System (MPU6050 + MMC5603).

---

## Voraussetzungen

### MATLAB Toolboxes

| Toolbox | Zweck |
|---|---|
| Sensor Fusion and Tracking Toolbox | `ahrsfilter`, `imufilter`, `magcal` |
| Aerospace Toolbox | `eulerd()`, quaternion-Konvertierung |
| Signal Processing Toolbox | Filteranalyse, Spektrum |
| (optional) Simulink + Aerospace Blockset | Blockdiagramm-basierte Fusion |

Installation via **Add-On Explorer** (`Home → Add-Ons → Get Add-Ons`).

### Hardware

- ESP32 NodeMCU-32S verbunden via USB (`/dev/cu.usbserial-110` auf macOS)
- ESP32 streamt Teleplot-Format mit 100 Hz:
  ```
  >imu/Acce_x: 0.12
  >imu/Acce_y: 10.07
  >imu/Acce_z: -0.75
  >imu/Gyro_x: 0.003
  >imu/Gyro_y: ...
  >imu/Gyro_z: ...
  >mag/x: 2167.0
  >mag/y: ...
  >mag/z: ...
  ```

---

## Port-Konflikt beheben

Bevor MATLAB den Serial-Port öffnen kann, muss PlatformIO Monitor geschlossen sein.

**Hängendes MATLAB-Objekt:**
```matlab
clear s
delete(instrfindall)
```

**Port von anderem Prozess gehalten (macOS Terminal):**
```bash
lsof | grep usbserial
kill -9 <PID>
```

---

## Dateien

| Datei | Funktion |
|---|---|
| `live_fusion.m` | Haupt-Script: Serial lesen → NED-Alignment → `ahrsfilter` → `poseplot` |
| `mag_calibration.m` | Magnetometer Hard/Soft-Iron-Kalibrierung via `magcal` |
| `print_axes.m` | Konsolen-Debug: Accel/Gyro/Mag-Rohwerte per `fprintf` ausgeben |

---

## Funktions-Referenz

### `live_fusion.m`

Liest Teleplot-Zeilen vom Serial-Port, aligned Achsen auf NED, füttert `ahrsfilter` und zeigt Orientation in `poseplot`.

```matlab
fs = 100;
s  = serialport("/dev/cu.usbserial-110", 115200);
configureTerminator(s, "LF");
flush(s);

FUSE   = ahrsfilter('SampleRate', fs);
viewer = poseplot;

accel = zeros(1,3);
gyro  = zeros(1,3);
mag   = zeros(1,3);

while true
    line = readline(s);

    if contains(line, "Acce_x"), accel(1) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_y"), accel(2) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Acce_z"), accel(3) = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_x"), gyro(1)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_y"), gyro(2)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "Gyro_z"), gyro(3)  = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/x"),  mag(1)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/y"),  mag(2)   = sscanf(extractAfter(line,": "), "%f"); end
    if contains(line, "mag/z")
        mag(3) = sscanf(extractAfter(line,": "), "%f");

        % NED-Alignment: Gravitation liegt auf Accel_Y → [Y, X, Z]
        accel_ned = [accel(2), accel(1), accel(3)];
        gyro_ned  = [gyro(2),  gyro(1),  gyro(3)];
        mag_t     = mag * 1e-6;          % µT → T

        q = FUSE(accel_ned, gyro_ned, mag_t);
        viewer.Orientation = q;
    end
end
```

**NED-Mapping:** Gravitation liegt auf `Accel_Y ≈ +10.07` → Y ist physisch "unten" → NED-Down-Achse. Vorzeichen und Reihenfolge bei geänderter Montage anpassen.

---

### Quaternion ausgeben

`fprintf` akzeptiert keine Quaternion-Objekte direkt:

```matlab
qc = compact(q);                          % → [w, x, y, z] als double
fprintf("q: %.4f %.4f %.4f %.4f\n", qc(1), qc(2), qc(3), qc(4));
```

Euler-Winkel in Grad:

```matlab
eul = eulerd(q, 'ZYX', 'frame');          % → [Yaw, Pitch, Roll]
fprintf("Yaw: %.2f  Pitch: %.2f  Roll: %.2f\n", eul(1), eul(2), eul(3));
```

Relevanter Winkel für Ruderblatt-Analyse ist **Yaw** (Rotation um Z).

---

### `mag_calibration.m`

Sensor durch alle Orientierungen rotieren (Kugel-Bewegung), Rohdaten sammeln, dann:

```matlab
[A, b, expmfs] = magcal(raw_mag_data);    % Hard/Soft-Iron-Korrektur
mag_calibrated = (raw_mag - b) * A;
```

`raw_mag_data`: N×3 Matrix in µT. Kalibrierparameter `A` und `b` speichern und in `live_fusion.m` vor der Übergabe an `ahrsfilter` anwenden.

---

### `print_axes.m`

Konsolen-Ausgabe für Achsen-Alignment-Verifikation ohne Plot:

```matlab
fprintf("Accel [m/s²]: X=%.3f  Y=%.3f  Z=%.3f\n", accel(1), accel(2), accel(3));
fprintf("Gyro  [rad/s]: X=%.3f  Y=%.3f  Z=%.3f\n", gyro(1),  gyro(2),  gyro(3));
fprintf("Mag   [µT]:    X=%.1f  Y=%.1f  Z=%.1f\n", mag(1),   mag(2),   mag(3));
```

Erwartete Ruhewerte: `Accel_Y ≈ +10.07`, `Accel_X/Z ≈ 0`, `Mag_X ≈ 2167 µT` dominant.

---

## Bekannte Einschränkungen

- MATLAB hat kein fertiges Sensor-Objekt für MPU6050 + externen MMC5603 (nur für MPU9250). Rohdaten werden manuell per Serial eingelesen.
- `ahrsfilter` und `imufilter` erwarten Magnetometer-Eingabe in **Tesla** (nicht µT) → Faktor `1e-6`.
- Gyro muss in **rad/s** übergeben werden. ESP32-Code muss alle drei Gyro-Achsen streamen.
- `poseplot` aktualisiert nur im `while`-Loop; Ctrl+C beendet die Schleife und gibt den Port frei.