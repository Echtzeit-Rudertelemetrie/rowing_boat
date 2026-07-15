// ─────────────────────────────────────────────────────────────────────────────
// test_imu_stream_s3.cpp — ICM-20948-Rohdaten-Stream fuer die MATLAB-Skripte
// in ../matlab_simulationen (main_test_S3.m, sensor_noise_calibration.m, ...).
//
// Env: xiao_s3_matlab (platformio.ini). Zeilenformat wie die alte MPU6050-
// Firmware (lib/Gyroscope in hall_wt), auf das die Skripte parsen:
//     >imu/Acce_x: 1.2345
// Einheiten ebenfalls wie damals (Adafruit-Konvention):
//     Accel m/s², Gyro rad/s, Mag µT.
// ekf_update.m normiert Accel/Mag (nur Richtung zaehlt), nutzt den Gyro aber
// roh in der Praediktion -> der MUSS in rad/s ankommen.
//
// mag/z wird als LETZTE Zeile eines Samples gesendet — darauf triggern die
// Skripte den EKF-Schritt / die Sample-Uebernahme.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>

static constexpr bool AD0_VAL = 1; // wie AngleReader: SparkFun-Breakout, Adresse 0x69
static constexpr unsigned long SAMPLE_INTERVAL_MS = 10; // 100 Hz = AK09916-Maximum
static constexpr float kDpsToRadS = 0.017453293f;       // dps -> rad/s
static constexpr float kMgToMs2 = 9.80665e-3f;          // mg  -> m/s²

ICM_20948_I2C icm;

void setup() {
    Serial.begin(115200);
    delay(200);

    Wire.begin();
    Wire.setClock(400000);

    // Wie AngleReader::begin(): der Chip braucht manchmal einen zweiten Anlauf.
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (attempt > 0) {
            delay(100);
        }
        ok = (icm.begin(Wire, AD0_VAL) == ICM_20948_Stat_Ok);
    }
    if (!ok) {
        // Dauerschleife mit Meldung statt Absturz — die Zeile matcht kein
        // MATLAB-Pattern und ist im Serial-Monitor sofort als Fehler sichtbar.
        while (true) {
            Serial.println("Failed to find ICM-20948 chip");
            delay(1000);
        }
    }

    // Identische Sensor-Konfiguration wie AngleReader::begin(), damit MATLAB-
    // Ergebnisse (Rauschvarianzen, magcal-A/b) auf die Produktion uebertragbar sind.
    ICM_20948_fss_t fss;
    fss.a = gpm8;
    fss.g = dps500;
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d23bw9_n34bw4;
    dlp.g = gyr_d23bw9_n35bw9;
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);
}

void loop() {
    static unsigned long lastSampleMs = 0;

    unsigned long nowMs = millis();
    if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS || !icm.dataReady()) {
        return;
    }
    lastSampleMs = nowMs;

    icm.getAGMT();

    Serial.printf(">imu/Acce_x: %.4f\n", icm.accX() * kMgToMs2);
    Serial.printf(">imu/Acce_y: %.4f\n", icm.accY() * kMgToMs2);
    Serial.printf(">imu/Acce_z: %.4f\n", icm.accZ() * kMgToMs2);
    Serial.printf(">imu/Gyro_x: %.4f\n", icm.gyrX() * kDpsToRadS);
    Serial.printf(">imu/Gyro_y: %.4f\n", icm.gyrY() * kDpsToRadS);
    Serial.printf(">imu/Gyro_z: %.4f\n", icm.gyrZ() * kDpsToRadS);

    // AK09916 in den Accel/Gyro-Frame gedreht (x, -y, -z) — exakt wie
    // AngleReader::readSample(). Neue magcal-Daten (A/b) werden damit direkt
    // in diesem Frame gelernt, siehe TODO oben in AngleReader.cpp.
    Serial.printf(">imu/mag/x: %.4f\n", icm.magX());
    Serial.printf(">imu/mag/y: %.4f\n", -icm.magY());
    Serial.printf(">imu/mag/z: %.4f\n", -icm.magZ()); // letzte Zeile = Sample komplett
}
