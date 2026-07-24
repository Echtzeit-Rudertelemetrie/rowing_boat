# Protokoll: EKF-Winkel-Debugging (Dolle, XIAO S3 + ICM-20948)

Session vom 22.07.2026, Worktree `angle_ekf__wt` (Branch `sensor_miniesp_code`).
Zusammenfassung des Chat-Verlaufs zur Übergabe.

---

## 1. Ausgangsproblem

- Sensor misst mit 100 Hz, ~45 Schläge/min, 9-Achsen-IMU (ICM-20948) mit Quaternion-EKF.
- Beobachtung: Bei Schlägen im ~90°-Bereich schwankt der Winkel stark; anschließend
  kehrt er mit exponentieller („logarithmischer") Kurve über **Minuten** zum
  Initialwert zurück.
- Ursprüngliche Vermutung: Kalman-Berechnungen zu langsam, Rechenstau.

**Befund: Kein Rechenstau.** Ein EKF-Schritt (4×4-/6×4-Matrizen, eine 6×6-Inversion)
braucht auf der S3-FPU deutlich unter 100 µs bei 10 ms Budget. Ein Stau würde
Lag/Jitter erzeugen, keine glatte Exponentialkurve. Wurde per Messung bestätigt
(„die Berechnungen gehen sehr schnell").

## 2. Nebenbefund: Teleplot-Ausgabe in DataSender.cpp

Die auskommentierte Schleife hatte drei Fehler:

1. `sizeof(angleBuffer)` liefert **Bytes** (16), nicht Elemente (8) → Out-of-bounds.
2. `%.2f` mit `uint16_t`-Argument → undefiniertes Verhalten (printf erwartet double).
3. `angleBuffer` enthält **quantisierte Codes** (0…65535 für −180…+180°), keine Grad.

Empfehlung: pro Sample `Serial.printf(">winkel:%.2f\n", data_->degreeSensor);`
direkt neben der Quantisierung, oder dequantisiert über
`ANGLE_MIN_DEG + code * (Spannweite/65535)`.

## 3. Analyse des eigentlichen Problems

### Rauschwerte waren Stillstandswerte

- `calibrateSensorNoise()` misst das Sensorrauschen in Ruhe (Gyro-Varianz ~1e-6 (rad/s)²).
- Als Prozessrauschen Q eingesetzt heißt das: Der Filter hält die Gyro-Integration
  für quasi perfekt → Kalman-Gain ~1e-4/Schritt → aufgelaufene Fehler brauchen
  Minuten zur Korrektur (die beobachtete Exponentialkurve).
- Als Messrauschen R (Accel ~8e-7) heißt es: Der Filter vertraut der Accel-Richtung
  auf ~0,1 % — während des Schlags misst der Accel aber Schwerkraft **plus**
  Linear-/Zentripetalbeschleunigung → Schwankungen.

### Beobachtbarkeits-Kernproblem (bleibt bestehen!)

Die Dollen-Drehachse ist vertikal → Schwerkraft liegt **auf** der Drehachse →
der Accelerometer ist für genau diesen Winkel prinzipbedingt blind.
**Einziger Sensor mit absoluter Winkelinformation um diese Achse: das Magnetometer.**

## 4. Eingebaute Änderungen — und deren Revert

Drei Schritte wurden implementiert (in `AngleReader.*` und `orientation_ekf.*`):

1. **Profiling**: `micros()` um `ekf.update()`, max |gyro| (Clipping-Check bei
   ±500 dps Full-Scale), Gating-Zähler; Ausgabe alle 5 s, später auf
   Teleplot-Kanäle umgestellt (`>ekf_mean_us`, `>gyro_max_dps`, `>gate_a`, …).
2. **Q-Anhebung**: gemessene Gyro-Varianz ×100, Floor 1e-4 (rad/s)² —
   Korrektur-Zeitkonstante sinkt von Minuten auf Sekunden.
3. **Bewegungs-Gating**: weicht |accel| >10 % (Mag: >20 %) vom Ruhe-Betrag ab,
   wird das jeweilige R stark aufgeblasen (zuletzt: absolutes R statt ×1000,
   weil ×1000 auf das winzige Ruhe-R immer noch ~1,6° Vertrauen bedeutete —
   die Zentripetalbeschleunigung zog sonst systematisch pro Schlag an der Basislinie).

**Wirkung:** Korrektur deutlich schneller; aber damit wurde sichtbar, dass der
Filter auch **gehaltene** Positionen (z. B. 90° stillgehalten) zurück auf die
Initialposition zieht — jetzt in Sekunden statt unbemerkt langsam.

**Alle Änderungen wurden auf Wunsch per `git restore` zurückgesetzt.**
Der Code steht wieder auf dem Commit-Stand; anschließend bestätigte sich das
langsame Zurückwandern auch im Originalzustand.

## 5. Diagnose der Wurzelursache

Wenn der Filter bei physisch gehaltenen 90° zurückzieht, meldet die Mag-Kette
effektiv „keine Drehung". Kandidaten (nach Wahrscheinlichkeit):

1. **Hard-Iron-Kalibrierung veraltet** — `MAG_B` (≈50/−89/−73 µT) ist so groß wie
   das Erdfeld; passt sie nicht mehr zum Aufbau, dominiert ein konstanter
   Restvektor und die kalibrierte Richtung ändert sich beim Drehen kaum.
2. **Stale Mag-Daten** (AK09916 hinter dem I2C-Master des ICM-20948 eingefroren).
3. **Frame-/Vorzeichenfehler** im Mapping (offener TODO in `readSample()`).
4. **Magnetische Umgebung** (Stahltisch, Monitor …).

Test ohne Code-Änderung: Env `xiao_s3_matlab` flashen (streamt `>imu/mag/x|y|z`
roh), Dolle langsam 360° drehen, Kanäle beobachten (eingefroren/Amplitude/Mittelwerte
vs. `MAG_B`).

## 6. Mag-Neukalibrierung (`../matlab_simulationen/magnetometer_calibrate.m`)

- **Bug behoben:** verirrtes `clear` mitten in der Plot-Zeile löschte nach der
  Aufnahme den kompletten Workspace (A/b weg, Skript bricht ab).
- Stolperfallen: Teleplot muss den Seriellport freigeben; auf dem Board muss
  `xiao_s3_matlab` laufen, sonst hängt die Aufnahmeschleife; `magcal()` braucht
  die Sensor Fusion/Navigation Toolbox.
- **Erster Kalibrierversuch verworfen:** Punktwolke war ein schmales Band
  (nur eine Drehachse, außerdem `num_points = 1000` = nur 10 s bei 100 Hz) →
  Ellipsoid-Fit degeneriert, A/b unbrauchbar.
- Vorgehen für gültige Kalibrierung:
  - `num_points ≈ 6000` (~60 s),
  - Sensor um **alle** Achsen taumeln bis die Kugelschale im Live-Plot geschlossen ist,
  - Abstand zu Metall, **in Einbaulage/mit Halterung**,
  - Residuum < 2–3 %, `expMFS` ≈ 48–50 µT plausibel,
  - neue `A`/`b` nach `AngleReader.cpp` (`MAG_A`/`MAG_B`), `xiao_s3` flashen,
    90°-Haltetest wiederholen.

Frames sind konsistent: Der MATLAB-Stream sendet bereits (X, −Y, −Z) — dasselbe
Frame, auf das die Firmware `MAG_B` anwendet; `(raw − b) * A` entspricht exakt
`calibrateMag()`.

## 7. Offene Punkte / nächste Schritte

- [ ] Mag-Neukalibrierung mit Vollabdeckung durchführen und Ergebnis eintragen.
- [ ] 90°-Haltetest: Position muss stehen bleiben.
- [ ] Danach entscheiden: Q-Anhebung + Bewegungs-Gating erneut einbauen
  (waren fachlich richtig, machten nur den Mag-Fehler sichtbar).
- [ ] Prüfen: Gyro-Clipping bei ±500 dps während echter Schläge (`gyro max`
  beobachten; ggf. Full-Scale auf `dps1000`).
- [ ] Offener TODO: Mag-/Frame-Vorzeichen am realen Aufbau prüfen.
- [ ] Langfristig: Gyro-Bias als EKF-Zustand (x = [q; b], F 7×7 mit Kopplung
  −0.5·W_mat(q)·dt, H-Spalten für b = 0, Bias wird über die P-Korrelation
  mitgeschätzt) — erlaubt kleines Q bei schneller Drift-Korrektur.

## 8. Weitere Erkenntnisse

- 200 Hz Abtastung brächte wenig: Ein-Achsen-Rotation kommutiert, die Fehler sind
  Amplitudenfehler (Clipping, gestörte Messungen), keine Diskretisierungsfehler.
  Falls doch: Prädiktion+Accel 200 Hz, Mag bleibt bei 100 Hz (AK09916-Limit).
- Teleplot zeigt nur `>name:wert`-Zeilen — Klartext-Debugausgaben gehen unter;
  Diagnose-Ausgaben daher immer im Teleplot-Format.
