#pragma once

#include <Arduino.h>
#include <ICM_20948.h>
#include "orientation_ekf.h"

#define SCL 6
#define SDA 5

class AngleReader {
public:
    AngleReader();
    // Initialisiert den ICM-20948 (Accel + Gyro + AK09916-Magnetometer, ein Chip).
    // MUSS nach Serial.begin()/Wire.begin() aufgerufen werden, nicht im Konstruktor.
    // Gibt false zurueck, wenn der Sensor nicht gefunden wurde (blockiert nicht).
    // Erfasst bei Erfolg die Gyro-Ruhe-Offsets -> Sensor beim Boot ruhig halten!
    bool begin();
    float sampleAndCalculateAngle();

private:
    // Nominale Abtastperiode: 10 ms = 100 Hz (Maximum des AK09916). Den Takt
    // gibt der 100-Hz-Timer in DollenApp vor; die halbe Periode dient hier nur
    // noch als Burst-Schutz-Schwelle gegen Mini-dt-Schritte bei Queue-Rueckstau.
    static constexpr unsigned long SAMPLE_INTERVAL_US = 10000UL;
    static constexpr bool AD0_VAL = 1; // SparkFun-Breakout: ADR-Jumper offen -> I2C-Adresse 0x69
    static constexpr int GYRO_CALIB_SAMPLES = 500;

    static const float MAG_A[3][3];
    static const float MAG_B[3];

    // Default-Rauschwerte (Fallback, falls die Boot-Kalibrierung fehlschlaegt
    // oder der Sensor dabei nicht still lag) — aus sensor_noise_calibration.m.
    static constexpr float kDefaultGyroNoise  = 9.89699e-07f; // (rad/s)^2, roh
    static constexpr float kDefaultAccelNoise = 8.16387e-07f; // normierter Accel-Vektor
    static constexpr float kDefaultMagNoise   = 3.11544e-05f; // normierter Mag-Vektor

    static constexpr int NOISE_CALIB_SAMPLES = 200;
    // Grobe obere Schranke: liegt eine gemessene Varianz darueber, war der
    // Sensor waehrend der Kalibrierung vermutlich in Bewegung -> verwerfen.
    static constexpr float kNoisePlausibilityLimit = 1e-2f;

    // ── TODO/REFACTOR: Kalibrierung raus aus AngleReader ─────────────────────
    // Die drei calibrate*()-Methoden gehoeren in eine eigene Klasse mit eigener
    // main + eigener PlatformIO-Env (analog xiao_s3_matlab), nicht in den
    // 100-Hz-Messpfad. Gruende:
    //   * Kalibrierung ist ein EINMALIGER Werksschritt, kein Boot-Schritt. Aktuell
    //     kostet sie bei jedem Start ~6 s (2x 3 s Deadline) und unterstellt, dass
    //     die Dolle beim Einschalten still liegt — im Boot ist das nicht gegeben.
    //   * Ergebnisse gehoeren nach NVS (Preferences), nicht in den Flash-Code:
    //     Rauschvarianzen je Achse, Gyro-Offsets UND MAG_A/MAG_B, die heute als
    //     Konstanten oben im .cpp stehen. Boot liest NVS, faellt bei leerem NVS
    //     auf die kDefault*-Werte zurueck.
    //   * Env-Vorschlag: xiao_s3_calib mit -D RUN_FACTORY_CALIB=1 und eigener
    //     main, die die Messung fuehrt (Nutzerfuehrung ueber Serial) und schreibt.
    // Siehe auch das Zeroing-Thema: setReferences() definiert den Nullpunkt aus
    // der Boot-Lage — ein definiertes Zeroing (Ruder in Referenzposition, dann
    // Kommando) gehoert in dieselbe Kalibrier-Main.
    //
    // Sensor-Rauschen im Stillstand messen und bei plausiblem Ergebnis ins
    // EKF uebernehmen (sonst bleiben die kDefault*Noise-Werte aktiv).
    void calibrateSensorNoise();

    static void calibrateMag(const float raw[3], float out[3]);
    void calibrateGyroOffsets();
    bool readSample(Vec3& gyroV, Vec3& accelV, Vec3& magV);

    ICM_20948_I2C icm;
    OrientationEKF ekf;
    Quat q;
    bool initialized;
    bool hwOk;
    unsigned long lastUpdateUs;
    float lastAngleDeg;
    float gyroOffset[3]; // rad/s, bei Boot in Ruhe erfasst
};
