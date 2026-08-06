# Boot-IMU kalibrieren und Achse prüfen

Die Boot-IMU verwendet denselben Quaternion-EKF, die Gyro-Bias-Ermittlung,
Stillstandserkennung sowie Beschleunigungs- und Magnetometer-Gates wie die
Dollen-IMU. Ihre Magnetometer-Kalibrierung ist separat, weil Einbaulage,
Gehäuse, Kabel und nahe Elektronik das Magnetfeld verändern.

## 1. Vorbereitung

Den kompletten Hub mit IMU, Gehäuse, Kabeln und der später verwendeten
Elektronik zusammenlassen. Für die 3D-Kalibrierung muss diese Einheit vor dem
festen Einbau um alle drei Achsen gedreht werden können. Abstand zu Werkzeug,
Lautsprechern, Stahlgestellen und anderen Magneten halten.

```bash
cd ~/Documents/rudern/rowing_boat
python3 -m pip install -r scripts/requirements-angle-tools.txt
```

## 2. Diagnose-Firmware flashen

```bash
~/.platformio/penv/bin/pio run -e boat_imu_diag -t upload
```

Nach dem Reset die Einheit mindestens sechs Sekunden absolut ruhig und in
ihrer normalen Einbaulage liegen lassen. In dieser Zeit wird der Gyro-Bias
bestimmt.

## 3. Drehachse und Vorzeichen prüfen

```bash
~/.platformio/penv/bin/pio device monitor -e boat_imu_diag
```

Die Einheit wie das Boot flach halten und nur um die physische Hochachse
drehen. Der Einbautest vom 27.07.2026 hat `pitch_deg` als dominante Achse
bestätigt: Eine flache Drehung um etwa 42 Grad änderte Pitch um etwa 42 Grad,
während Yaw und Roll nahe null blieben. Der gemeinsame Praxistest zeigte
entgegengesetzte Vorzeichen beider Einbaulagen. Die App addiert deshalb
Boot-Pitch zum rohen Dollenwinkel, wodurch sich die gemeinsame Drehung aufhebt.

## 4. Magnetometerdaten aufnehmen

```bash
python3 scripts/record_angle_csv.py \
  --port '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:C6:D4:78-if00' \
  --baud 460800 \
  --duration 120 \
  --output logs/boat_mag_full_3d.csv
```

Während der 120 Sekunden die komplette Einheit langsam und gleichmäßig durch
möglichst viele Orientierungen bewegen: um jede der drei Achsen mehrfach
drehen, kippen und taumeln. Jede Raumrichtung soll erreicht werden.

## 5. Kalibrierung berechnen

```bash
MPLCONFIGDIR=/tmp/matplotlib-angle \
python3 scripts/calibrate_magnetometer.py logs/boat_mag_full_3d.csv \
  --output-dir calibration/boat_mag_01
```

Der Befehl prüft 3D-Abdeckung und Fit-Residuum. Der erzeugte Block aus
`calibration/boat_mag_01/mag_calibration_constants.txt` ersetzt den `mag`-Block
des Boot-Hubs in der Einheitentabelle in `lib/UnitIdentity/UnitIdentity.cpp`.
Der Hub wird dort über seine Board-MAC gefunden; ein eigenes Konfigurationsfile
(früher `lib/Imu/BoatImuConfig.h`) gibt es nicht mehr.

## 6. Produktions-Firmware und App

Nach dem Eintragen der Konstanten:

```bash
~/.platformio/penv/bin/pio run -e prod_BLE_sender -t upload
cd ~/Documents/rudern/rudertelemetrie-mobile-app
~/flutter/bin/flutter run
```

Bei jedem Einschalten Boot und Dolle während der ersten sechs Sekunden ruhig
halten. In der App wird der Dollenwinkel anschließend als
`Dollenwinkel + Boot-Pitch` berechnet und auf den Bereich -180 bis +180 Grad
abgebildet.
