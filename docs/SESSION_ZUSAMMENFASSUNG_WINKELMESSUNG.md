# Übergabe: Überarbeitung der Winkelmessung

Stand: 25. Juli 2026

Diese Zusammenfassung erklärt dem ursprünglichen Implementierer kompakt, warum
die Winkelmessung geändert wurde und wie die neue Lösung arbeitet. Die
ausführliche Bedienungs-, Kalibrierungs- und Testanleitung steht in
`docs/WINKELMESSUNG_KALIBRIERUNG_UND_TEST.md`.

## Ausgangslage

Die bisherige Winkelmessung driftete nach größeren Bewegungen zunächst sichtbar
weiter. Der Drift nahm erst über Minuten ab und endete teilweise bei einem
stabilen, aber falschen Winkel. Wesentliche Ursachen waren eine wenig robuste
Gyro-Kalibrierung, die ungeprüfte Verwendung von Sensorrauschwerten und
Magnetometerkorrekturen, fehlendes Gating bei dynamischer Bewegung sowie
unzureichende Absicherung von Timing und EKF-Mathematik.

Die bestehende Architektur und die physische Achsenabbildung wurden beibehalten:
Der ICM-20948 wird vom Seeed Studio XIAO ESP32S3 gelesen, intern als Quaternion
verarbeitet und die bisher als `roll` verwendete Nutzachse ausgegeben. Die
Produktionsumgebung bleibt `xiao_s3`.

## Was geändert wurde

- Die IMU-Verarbeitung läuft nachvollziehbar mit 100 Hz und verwendet reale
  Mikrosekunden-Zeitstempel. Unplausible oder zu große `dt`-Werte werden erkannt
  und nicht als lange Integration eines einzelnen Gyrowerts verwendet.
- Die Boot-Kalibrierung ist jetzt zeitbasiert: Aufwärmphase, mehrere Sekunden
  Messzeit, Mindestanzahl von Samples sowie Prüfungen von Gyro-Mittelwert,
  Gyro-Varianz, Beschleunigungsbetrag und Bewegung. Ungültige Kalibrierungen
  werden nicht stillschweigend übernommen.
- Eine Stationärerkennung verwendet ein Zeitfenster statt einer
  Einzelmessung. Sie berücksichtigt Winkelgeschwindigkeit, Gyro-Streuung,
  Beschleunigungsbetrag und dessen Stabilität.
- Der Gyro-Bias wird nach dem Boot langsam nachgeführt, aber ausschließlich in
  sicher erkannten Ruhephasen. Lernrate und maximale Biasänderung sind begrenzt;
  eine doppelte Offsetsubtraktion wird vermieden.
- Beschleunigungsdaten korrigieren die Lage nur, wenn ihr Betrag plausibel ist.
  Hysterese verhindert ein Flackern des Flags `accel_valid` bei einzelnen
  Ausreißern.
- Magnetometerdaten werden auf Freshness, Overflow und plausiblen Feldbetrag
  geprüft. Die vorhandenen `MAG_A`-/`MAG_B`-Werte waren nicht verifiziert und
  verursachten beim 90°-Haltetest ein langsames Zurückziehen zu einer falschen
  Referenz. Deshalb ist die Magnetometerkorrektur derzeit bewusst deaktiviert,
  bis eine echte Hard-/Soft-Iron-Kalibrierung durchgeführt und mit
  `MAG_CALIBRATION_VERIFIED` freigegeben wurde.
- Prozess- und Messrauschen besitzen dokumentierte Mindestwerte. Gemessene
  Stillstandsvarianzen werden nicht mehr ungeprüft als vollständige EKF-Q/R-
  Parameter verwendet.
- Der EKF schützt sich gegen ungültige Vektoren, NaN/Inf und degenerierte
  Quaternionen. Quaternionen werden normalisiert; das Kovarianzupdate verwendet
  die numerisch robustere Joseph-Form und stellt die Symmetrie wieder her.
- Queue-Tiefe, verlorene Events, Timing-Ausreißer, Sensor-Gültigkeit,
  Stationarität, Bias und Kalibrierungszustand sind diagnostizierbar.
- ESP-NOW sendet jedes Paket nur einmal. Die früheren Wiederholungen derselben
  Sequenznummer erzeugten Duplikate und rückwärts springende Zeitachsen in der
  App.

Die zentralen, hardwareabhängig abzustimmenden Parameter befinden sich in
`lib/AngleReader/AngleReaderConfig.h`. Änderungen daran sollten nur anhand
aufgezeichneter Logs erfolgen.

## Neue Diagnosewerkzeuge

Das PlatformIO-Environment `xiao_s3_angle_diag` deaktiviert unnötige
Funktionen und gibt vollständige IMU-, Filter-, Bias-, Timing- und
Gültigkeitsdaten als CSV aus. Dazu gehören:

- `scripts/record_angle_csv.py` zur robusten seriellen Aufzeichnung,
- `scripts/analyze_angle_log.py` für Drift-, Timing- und Winkelauswertung,
- `scripts/calibrate_magnetometer.py` für eine Hard-/Soft-Iron-Auswertung,
- `test/host_orientation_test.cpp` für Schutz- und Mathematiktests,
- die Messergebnisse unter `reports/`.

## Bisherige Messergebnisse

Beim aufgezeichneten Stillstandstest über rund 10,9 Minuten wurden folgende
Werte erreicht:

- mittleres `dt`: 9,9997 ms,
- 99%-Perzentil: 10,0010 ms,
- stationärer Anteil: 99,12 %,
- linearer Stillstandsdrift: −0,000688 °/min,
- Winkel-Spitze-zu-Spitze: 0,5177 °,
- keine Gyro-Clippings.

Nach der Korrektur des Gatings und dem Abschalten der nicht verifizierten
Magnetometerkalibrierung zeigte der abschließende 90°-Haltetest in den letzten
60 Sekunden ungefähr 0,00275 °/min Drift. Diese Ergebnisse belegen eine sehr
gute Stillstands- und Haltestabilität auf der getesteten Hardware. Sie ersetzen
noch keine vollständige Aussage über absolute Winkelgenauigkeit,
Temperaturbereich oder magnetisch gestützte Langzeitgenauigkeit.

## Änderung in der mobilen App

Die App verwirft jetzt doppelte und verspätete Pakete, behandelt einen Neustart
des Senders kontrolliert und erzeugt eine monotone lokale Zeitachse. Winkel und
Kraft werden innerhalb derselben Sensorgruppe korrekt zugeordnet. Alle 61
Flutter-Tests liefen nach der Änderung erfolgreich durch.

Beim abschließenden Systemtest waren zunächst zwei physische Sender gleichzeitig
mit `ESP-ID 1` eingeschaltet. Das erzeugte abwechselnde Sequenznummern und
Nullwinkel in der App. Dies war kein Fehler der neuen Winkelberechnung. Nach dem
Ausschalten des versehentlich zusätzlich aktiven Geräts wurden die Winkelwerte
korrekt live dargestellt. Die produktive Winkelplatine verwendet deshalb
weiterhin die bestehende `ESP-ID 1`.

## Noch offen

Vor einer belastbaren absoluten Winkelabnahme sind weiterhin notwendig:

1. Magnetometer in der tatsächlichen Einbausituation aufzeichnen und per
   Hard-/Soft-Iron-Verfahren kalibrieren.
2. Die ermittelten Parameter eintragen, `MAG_CALIBRATION_VERIFIED` aktivieren
   und Magnetstörtests wiederholen.
3. Mehrere mechanisch reproduzierbare 0°-/90°-Tests bei Kalt- und Warmstart
   durchführen.
4. Bei gleichzeitig betriebenen Rudersensoren jedem physischen Sender eine
   eindeutige ESP-ID geben.

Die wichtigste Leitlinie lautet: Eine ruhige Anzeige allein gilt nicht als
Erfolg. Drift, absoluter Endfehler, Einschwingzeit, Wiederholbarkeit und
Störfestigkeit müssen getrennt ausgewertet werden.
