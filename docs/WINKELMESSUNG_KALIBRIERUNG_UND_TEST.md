# Winkelmessung: Kalibrierung, Diagnose und Abnahme

Diese Anleitung gilt für den Seeed Studio XIAO ESP32S3 mit ICM-20948. Sie setzt
kein Vorwissen über IMUs voraus. Die Produktionsfirmware heißt `xiao_s3`; für
alle Messungen und Kalibrierungen wird `xiao_s3_angle_diag` verwendet. Letztere
schaltet Kraftsensor, WLAN und ESP-NOW ab, verarbeitet die IMU mit 100 Hz und
sendet vollständige CSV-Zeilen mit 50 Hz bei 460800 Baud.

## 1. Vorbereitung

Im Projektverzeichnis:

```bash
cd /home/eron/Documents/rudern/rowing_boat
python3 -m venv .venv-angle
source .venv-angle/bin/activate
python3 -m pip install -r scripts/requirements-angle-tools.txt
```

Den XIAO per USB anschließen. Verfügbare Ports anzeigen:

```bash
ls -l /dev/serial/by-id/
```

Den vollständigen Pfad verwenden, der `Espressif_USB_JTAG_serial_debug_unit`
enthält. Falls `/dev/serial/by-id` nicht existiert:

```bash
python3 -m serial.tools.list_ports
```

In den folgenden Beispielen steht `<PORT>` für diesen Pfad.

## 2. Diagnosefirmware flashen

```bash
/home/eron/.platformio/penv/bin/pio run -e xiao_s3_angle_diag -t upload
```

Optional nur beobachten:

```bash
/home/eron/.platformio/penv/bin/pio device monitor \
  -e xiao_s3_angle_diag --port '<PORT>' --baud 460800
```

Nicht gleichzeitig Monitor und Aufzeichnungsskript öffnen; nur ein Programm
kann den seriellen Port besitzen.

## 3. Boot-Gyro-Kalibrierung

1. Die komplette Dolle mitsamt IMU auf eine feste, vibrationsfreie Unterlage
   legen oder in der normalen Einbaulage mechanisch festklemmen.
2. Nicht nur die Platine, sondern die komplette Baugruppe darf sich nicht
   bewegen. Tisch nicht berühren und Kabel nicht ziehen.
3. USB neu verbinden oder Reset drücken.
4. Ab `ANGLE_CAL,WARMUP` mindestens 5,5 Sekunden nicht berühren:
   1,5 Sekunden Aufwärmen plus 4 Sekunden Messung.
5. Erfolg sieht so aus:

   ```text
   ANGLE_CAL,QUALITY,...,result=VALID
   ANGLE_CAL,SUCCESS,bias_dps=...|...|...,accel_rest_mg=...
   ```

6. Bei `result=MOVEMENT` bleibt das Gerät liegen. Die Firmware wartet eine
   Sekunde und versucht es erneut. Nicht berühren, bis `SUCCESS` erscheint.
7. Nach drei Fehlschlägen erscheint
   `ANGLE_WARNING,BOOT_CALIBRATION_FAILED,using_zero_bias`. Dann Unterlage,
   Kabelzug und Vibrationen beseitigen und neu starten. Diesen Lauf nicht für
   eine Abnahme verwenden.

Die Grenzwerte stehen zentral in
`lib/AngleReader/AngleReaderConfig.h`. Sie dürfen erst nach mehreren Logs
geändert werden.

## 4. CSV aufzeichnen

Beispiel für zehn Minuten:

```bash
python3 scripts/record_angle_csv.py \
  --port '<PORT>' --baud 460800 \
  --duration 600 --output logs/still_cold_01.csv
```

Das Skript schreibt nur vollständige CSV-Zeilen. Statusmeldungen erscheinen im
Terminal, nicht in der CSV. Auswertung:

```bash
MPLCONFIGDIR=/tmp/matplotlib-angle \
python3 scripts/analyze_angle_log.py logs/still_cold_01.csv \
  --output-dir reports/still_cold_01
```

Ergebnis sind `metrics.txt`, `angle_diagnostics.png` und `dt_histogram.png`.

## 5. Physische Achse bestimmen

1. Diagnosefirmware starten und erfolgreiche Boot-Kalibrierung abwarten.
2. Gerät in der normalen Einbaulage fixieren.
3. Im seriellen Monitor `z` senden. `# ZERO_APPLIED` muss erscheinen.
4. Die Dolle langsam ausschließlich um ihre reale Gelenkachse drehen. Andere
   Kippbewegungen vermeiden.
5. Erwartung: `output_angle_deg` und `roll_deg` ändern sich deutlich und mit
   gleichem Vorzeichen; `pitch_deg` und `yaw_deg` bleiben wesentlich kleiner.
6. Nacheinander kleine positive Drehungen um die drei sichtbaren Platinenachsen
   durchführen und notieren, welcher Roh-Gyrokanal reagiert. Für die angenommene
   Montage gilt: Sensor-Y → EKF-X → Roll.
7. Falls die reale Gelenkbewegung hauptsächlich Pitch oder Yaw erzeugt, nicht
   blind die Ausgabe umstellen. Zuerst Einbaulage und die zyklische Abbildung
   `(Sensor Y, Sensor Z, Sensor X)` in `AngleReader::readSample()` korrigieren.
8. Ein falsches Vorzeichen kann später an einer einzigen klar dokumentierten
   Ausgabekonstante geändert werden; die Quaternion-Frames dürfen nicht durch
   willkürliche Einzelvorzeichen gespiegelt werden.

## 6. Mechanischen Nullpunkt setzen

Die interne Quaternion bleibt relativ zur Bootreferenz. Der Nutzwinkel besitzt
einen getrennten Nullpunkt:

1. Dolle in die mechanisch definierte Nullstellung bringen.
2. Mindestens zwei Sekunden ruhig halten.
3. Im Diagnosemonitor `z` senden.
4. `output_angle_deg` muss anschließend nahe 0° liegen.

Die Nullsetzung ist derzeit flüchtig und gilt bis zum Neustart. Die
Produktionsschnittstelle `Sensor::zeroAngle()` ist vorhanden; eine Bedienaktion
oder NVS-Speicherung kann später angebunden werden, sobald die mechanische
Nullprozedur festgelegt ist.

## 7. Magnetometer-Ellipsoidkalibrierung

Die ausgelieferten neutralen Konstanten sind absichtlich **keine** behauptete
Kalibrierung. Für die Kalibrierung muss die IMU in ihrer endgültigen Halterung
bleiben, weil Schrauben, Halter und Kabel das Feld beeinflussen.

### Umgebung

- Mindestens einen Meter Abstand zu Stahlmöbeln, Werkzeug, Lautsprechern,
  Motoren, Netzteilen, Monitoren, Telefonen und magnetischen Verschlüssen.
- Nicht auf einem Stahltisch arbeiten.
- Die Dolle während der Aufnahme nicht an einen anderen Ort tragen.
- Elektrische Verbraucher so betreiben wie später im Boot, soweit sie zur
  Baugruppe gehören.

### Aufnahme

1. Diagnosefirmware flashen.
2. Boot-Kalibrierung vollkommen ruhig abschließen.
3. Aufnahme für zwei Minuten starten:

   ```bash
   python3 scripts/record_angle_csv.py \
     --port '<PORT>' --baud 460800 --duration 120 \
     --output logs/mag_full_3d.csv
   ```

4. In den ersten zehn Sekunden ruhig bleiben.
5. Danach die gesamte Baugruppe langsam und kontinuierlich durch **alle**
   Orientierungen bewegen:
   - zehn volle Umdrehungen um die lange Platinenachse,
   - zehn volle Umdrehungen um die quer liegende Platinenachse,
   - zehn volle Umdrehungen um die senkrechte Platinenachse,
   - anschließend etwa 60 Sekunden taumeln: abwechselnd Vorderseite, Rückseite
     und jede der vier Kanten nach oben halten und dabei langsam weiterdrehen.
6. Keine schnellen Schläge ausführen. Ziel ist eine geschlossene 3D-Kugelschale,
   nicht eine einzelne Kreisbahn.

### Fit berechnen

```bash
MPLCONFIGDIR=/tmp/matplotlib-angle \
python3 scripts/calibrate_magnetometer.py logs/mag_full_3d.csv \
  --output-dir calibration/mag_01
```

Das Skript lehnt weniger als 500 Punkte, schlechte 3D-Abdeckung und ein
Fit-Residuum über 8 % ab. Angestrebt werden unter 3 %. Die Datei
`calibration/mag_01/mag_calibration_constants.txt` enthält:

```cpp
{
    true,
    {
        { ... }, { ... }, { ... },
    },
    { ... },
},
```

Dieser Block ersetzt den `mag`-Block der betroffenen Einheit in der
Einheitentabelle in `lib/UnitIdentity/UnitIdentity.cpp`. Die Einheit wird dort
über ihre Board-MAC gefunden, es ist also genau ein Eintrag zu ändern und keine
andere Datei anzufassen. Danach beide Environments neu bauen.
Die Werte werden derzeit im Firmware-Image gespeichert, nicht in NVS. Dies ist
absichtlich nachvollziehbar und versionskontrolliert. NVS lohnt sich erst, wenn
eine Bedienoberfläche Kalibrierungen eindeutig einem Gerät zuordnen kann.

### Kontrolle

1. Neue Diagnosefirmware flashen.
2. Nochmals 60 Sekunden in alle Richtungen drehen.
3. `mag_norm` soll weitgehend konstant bleiben; `mag_valid` darf bei normalen
   Drehungen nicht dauerhaft 0 werden.
4. Punktwolke erneut auswerten.
5. Danach den 90°-Haltetest durchführen. Kehrt der Winkel trotz mechanisch
   gehaltener Position zurück, ist die Kalibrierung oder magnetische Umgebung
   weiterhin ungeeignet.

## 8. Sechs verbindliche Hardwaretests

Für jeden Test kalten und warmen Lauf eindeutig benennen. Keine Parameter
zwischen Wiederholungen unprotokolliert ändern.

### Test 1 – Boot und zehn Minuten Stillstand

1. Gerät mindestens 30 Minuten stromlos lassen.
2. Festklemmen, Diagnosefirmware starten und 600 Sekunden aufzeichnen.
3. Nicht berühren.
4. Mindestens dreimal wiederholen.
5. Bestehen: Kalibrierung `VALID`, keine NaN/Inf, `invalid_dt_count=0`,
   `stationary` nach etwa zwei Sekunden aktiv und Drift nach Warm-up <0,1°/min.

### Test 2 – Kontrollierte 90°-Drehung

1. Mechanischen Winkelanschlag, Anschlagwinkel oder digitale Winkelreferenz
   verwenden.
2. In Startposition `z` senden und 30 Sekunden warten.
3. In etwa zwei Sekunden auf +90° drehen.
4. Fünf Minuten mechanisch bei genau 90° halten.
5. Zurück auf 0°, danach bei −90° wiederholen.
6. Bestehen: Endfehler Ziel ±1°, kein minutenlanger Rücklauf, Einschwingen in
   wenigen Sekunden. Sollwinkel und echte Referenzgenauigkeit dokumentieren.

### Test 3 – Mehrere schnelle Bewegungen

1. 30 Sekunden Ruhe aufzeichnen.
2. Zehn typische Ruderschläge ausführen.
3. Fünf Minuten in einer bekannten Endposition fixieren.
4. Bestehen: `accel_valid` wird während dynamischer Abschnitte zeitweise 0;
   `gyro_clip_count` bleibt 0 oder führt zur begründeten Umstellung auf
   ±1000°/s; kein dauerhafter großer Endversatz.

### Test 4 – Magnetische Störung

1. Gerät ruhig fixieren und 30 Sekunden Basislinie aufnehmen.
2. Einen Stahlgegenstand langsam bis etwa 10 cm annähern, ohne das Gerät zu
   bewegen. Keine starken Permanentmagnete verwenden; sie können remanente
   Magnetisierung verursachen.
3. Gegenstand 20 Sekunden halten und wieder entfernen.
4. Bestehen: `mag_norm` verlässt die Grenzen, `mag_valid` wird 0, Winkel springt
   nicht abrupt; nach Entfernen wird Mag erst nach mehreren guten Samples wieder
   zugeschaltet und der Winkel erholt sich kontrolliert.

### Test 5 – Kaltstart und Warmstart

1. Test 1 kalt durchführen.
2. Gerät danach 20 Minuten eingeschaltet lassen.
3. Reset drücken, ohne Lage oder Umgebung zu ändern, und erneut zehn Minuten
   loggen.
4. Bias und Temperaturplots vergleichen.
5. Bestehen: Biasunterschied sichtbar dokumentiert; stationäre Online-Korrektur
   konvergiert ohne Winkelratschen; Wiederholbarkeit erreicht den Zielbereich.

### Test 6 – Timingstörung

1. Diagnosebetrieb starten.
2. Für einen reproduzierbaren Softwaretest vorübergehend einmalig `delay(100)`
   unmittelbar vor `sampleAndCalculateAngle()` in der Diagnose-Loop einfügen
   oder den USB-Ausgabepfad gezielt blockieren. Diese Änderung nie in Produktion
   übernehmen.
3. Bestehen: `invalid_dt_count` steigt, der Winkel macht keinen Sprung
   entsprechend `gyro_now × 100 ms`, danach folgen wieder gültige 10-ms-Schritte.
4. Produktionsbetrieb zusätzlich mit `xiao_s3_debug` prüfen:

   ```bash
   /home/eron/.platformio/penv/bin/pio run -e xiao_s3_debug -t upload -t monitor
   ```

   `ANGLE_TIMING` zeigt `queue_max` und `dropped`. Im normalen Betrieb muss
   `dropped=0` bleiben.

## 9. Produktionsfirmware

Nach erfolgreicher Magnetometerkalibrierung und Hardwareabnahme:

```bash
/home/eron/.platformio/penv/bin/pio run -e xiao_s3 -t upload
```

Die Produktionsfirmware vermeidet kontinuierliche CSV-Ausgabe. Für
Timingtelemetrie und vollständige Paketdiagnose wird ausschließlich
`xiao_s3_debug` verwendet, weil umfangreiche USB-Ausgabe selbst Task-Jitter
erzeugen kann.

## 10. Abnahmewerte

Getrennt dokumentieren:

| Eigenschaft | Kennzahl | Richtwert |
|---|---|---:|
| Genauigkeit | Endfehler bei fixierten ±90° | ≤ ±1° |
| Kurzzeitrauschen | Peak-to-Peak in 60 s Ruhe | aus Logs bestimmen |
| Langzeitdrift | lineare Steigung nach Warm-up | < 0,1°/min |
| Einschwingzeit | Zeit bis dauerhaft ±1° | wenige Sekunden |
| Wiederholbarkeit | Differenz mehrerer Neustarts | ≤ ±1° |
| Timing | ungültige `dt`, Queueverluste | 0 im Normalbetrieb |
| Robustheit | Sprung bei Mag-Störung | kein dauerhafter großer Fehler |

Ohne Hardwarelog darf kein Zahlenfeld als bestanden markiert werden.
