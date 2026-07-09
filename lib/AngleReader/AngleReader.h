#pragma once

#include <Arduino.h>
#include <ICM_20948.h>
#include "orientation_ekf.h"

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
