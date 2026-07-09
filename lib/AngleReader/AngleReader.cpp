// ─────────────────────────────────────────────────────────────────────────────
// AngleReader.cpp — Orientierung aus dem ICM-20948 (Accel + Gyro + AK09916-Mag)
// ─────────────────────────────────────────────────────────────────────────────
#include "AngleReader.h"

// ── (1) Magnetometer-Kalibrierung ─────────────────────────────────────────────
// TODO: Kalibrierwerte gelten pro Chip! Die alten MMC5603-Werte (magcal/MATLAB)
// sind fuer den AK09916 im ICM-20948 unbrauchbar. Bis eine neue magcal-Messung
// vorliegt: Identitaet/Null = unkalibriert. Neue Daten im Body-aligned-Frame
// erfassen (also nach dem Y/Z-Flip in readSample()).
const float AngleReader::MAG_A[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

const float AngleReader::MAG_B[3] = {0.0f, 0.0f, 0.0f};

// Eigene float-Konstante statt Arduinos DEG_TO_RAD: das Makro ist ein
// double-Literal und zieht die ganze Rechnung in Software-double — die
// S3-FPU kann nur single precision, double kostet ein Vielfaches.
static constexpr float kDegToRad = 0.017453293f;

static Quat makeIdentityQuat() {
    Quat q0;
    q0.m[0][0] = 1.0f;
    return q0;
}

AngleReader::AngleReader()
    : ekf(0.09f /*gyro*/, 0.25f /*accel*/, 0.64f /*mag*/),
      q(makeIdentityQuat()),
      initialized(false),
      hwOk(false),
      lastUpdateUs(0),
      lastAngleDeg(0.0f),
      gyroOffset{0.0f, 0.0f, 0.0f} {
    // KEIN Hardware-Zugriff im Konstruktor! Der laeuft als globale Static-Init
    // noch vor setup()/Serial.begin(). I2C-Init passiert in begin().
}

bool AngleReader::begin() {
    // Der ICM-20948 braucht nach dem Einschalten manchmal einen zweiten Anlauf.
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (attempt > 0) {
            delay(100);
        }
        ok = (icm.begin(Wire, AD0_VAL) == ICM_20948_Stat_Ok);
    }

    if (!ok) {
        Serial.print("Failed to find ICM-20948 chip: ");
        Serial.println(icm.statusString());
        return false;
    }

    // Full-Scale wie beim alten MPU6050-Setup: +/-8 g, +/-500 dps
    ICM_20948_fss_t fss;
    fss.a = gpm8;
    fss.g = dps500;
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    // DLPF ~24 Hz — Gegenstueck zu MPU6050_BAND_21_HZ
    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d23bw9_n34bw4;
    dlp.g = gyr_d23bw9_n35bw9;
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);

    // Magnetometer: startupMagnetometer() in begin() setzt den AK09916 bereits
    // auf 100-Hz-Dauerbetrieb -> passt zu SAMPLE_INTERVAL_US.

    hwOk = true;
    calibrateGyroOffsets();
    return true;
}

// ── Gyro-Ruhe-Offsets beim Boot erfassen (Sensor muss dabei still liegen) ─────
void AngleReader::calibrateGyroOffsets() {
    Serial.println("Gyro-Offset-Kalibrierung (Sensor ruhig halten)...");

    float sum[3] = {0.0f, 0.0f, 0.0f};
    int collected = 0;
    const unsigned long deadlineMs = millis() + 3000; // Notausstieg, falls keine Daten kommen

    while (collected < GYRO_CALIB_SAMPLES && millis() < deadlineMs) {
        if (icm.dataReady()) {
            icm.getAGMT();
            sum[0] += icm.gyrX();
            sum[1] += icm.gyrY();
            sum[2] += icm.gyrZ();
            ++collected;
        }
        delay(1);
    }

    if (collected > 0) {
        for (int i = 0; i < 3; ++i) {
            gyroOffset[i] = (sum[i] / collected) * kDegToRad;
        }
    }

    Serial.printf("Gyro-Offsets [rad/s]: %.5f %.5f %.5f (%d Samples)\n",
                  gyroOffset[0], gyroOffset[1], gyroOffset[2], collected);
}

// ── Hilfsfunktion: Magnetometer kalibrieren ──────────────────────────────────
void AngleReader::calibrateMag(const float raw[3], float out[3]) {
    float c[3] = {
        raw[0] - MAG_B[0],
        raw[1] - MAG_B[1],
        raw[2] - MAG_B[2]
    };

    for (int i = 0; i < 3; ++i) {
        out[i] = c[0] * MAG_A[0][i] + c[1] * MAG_A[1][i] + c[2] * MAG_A[2][i];
    }
}

// ── Einen kompletten Messsatz lesen und in den EKF-Frame bringen ─────────────
bool AngleReader::readSample(Vec3& gyroV, Vec3& accelV, Vec3& magV) {
    if (!hwOk || !icm.dataReady()) {
        return false;
    }

    icm.getAGMT();

    // Gyro: SparkFun-Lib liefert dps -> rad/s (EKF-Erwartung), Ruhe-Offset abziehen
    float gx = icm.gyrX() * kDegToRad - gyroOffset[0];
    float gy = icm.gyrY() * kDegToRad - gyroOffset[1];
    float gz = icm.gyrZ() * kDegToRad - gyroOffset[2];

    // Accel bleibt in mg — der EKF normiert, nur die Richtung zaehlt
    float ax = icm.accX();
    float ay = icm.accY();
    float az = icm.accZ();

    // AK09916 -> Accel/Gyro-Frame: X gleich, Y und Z invertiert
    // (ICM-20948-Datenblatt Kap. 10.1 "Orientation of Axes")
    float magAligned[3] = {icm.magX(), -icm.magY(), -icm.magZ()};
    float magCal[3];
    calibrateMag(magAligned, magCal);

    // Einbaulage -> EKF-Frame: Breakout ist mit Y nach OBEN montiert.
    // Zyklische Vertauschung (y, z, x) = echte Rotation (det +1, anders als der
    // alte x<->z-Tausch, der eine Spiegelung war): die vertikale Body-Y-Achse
    // landet auf der EKF-X-Achse, deren Drehung (Roll) der Dollen-Winkel ist.
    // Das Mag macht dieselbe Vertauschung mit (sitzt auf demselben Chip).
    // TODO: Vorzeichen am realen Aufbau pruefen (Drehrichtung Catch -> Finish).
    gyroV  = vec3(gy, gz, gx);
    accelV = vec3(ay, az, ax);
    magV   = vec3(magCal[1], magCal[2], magCal[0]);
    return true;
}

// ── Hauptfunktion ─────────────────────────────────────────────────────────────
float AngleReader::sampleAndCalculateAngle() {
    unsigned long nowUs = micros();

    // Zwischen echten Samples (und ohne Hardware) den letzten Winkel liefern,
    // damit keine falschen 0-Grad-Werte in die Funkpakete wandern.
    if (!hwOk) {
        return lastAngleDeg;
    }

    // 1) Einmalige Initialisierung: erste Messung liefert die EKF-Referenzen
    if (!initialized) {
        Vec3 gyroV, accelV, magV;
        if (!readSample(gyroV, accelV, magV)) {
            return lastAngleDeg;
        }

        ekf.setReferences(accelV, magV);

        lastUpdateUs = nowUs;
        initialized = true;
        return lastAngleDeg;
    }

    // 2) Kein eigener 100-Hz-Begrenzer mehr: der Timer in DollenApp tickt jetzt
    // selbst mit 100 Hz und gibt die Abtastrate vor (1 Tick = 1 EKF-Schritt).
    // Der alte Limiter (Skip wenn < 10 ms) hat zusammen mit dem 200-Hz-Timer
    // jeden zweiten Aufruf verworfen; mit Timer-Jitter haette er bei exakt
    // 100-Hz-Ticks sogar zufaellig echte Samples verschluckt (9.9 ms -> Skip,
    // naechster Schritt dann mit 20 ms dt). Es bleibt nur ein Burst-Schutz:
    // arbeitet die Event-Queue einen Rueckstau ab, kommen Ticks quasi
    // gleichzeitig an — ein EKF-Schritt mit Mini-dt bringt nichts (der AK09916
    // haette ohnehin keine neuen Daten) und wird uebersprungen.
    if ((nowUs - lastUpdateUs) < SAMPLE_INTERVAL_US / 2) {
        return lastAngleDeg;
    }

    Vec3 gyroV, accelV, magV;
    if (!readSample(gyroV, accelV, magV)) {
        return lastAngleDeg;
    }

    // *1e-6f statt /1e6f: Division hat auf der LX7-FPU keinen eigenen Befehl
    // und wird zur mehrzykligen Reziprok-Sequenz — Multiplikation ist 1 Takt.
    float dt = (nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    // EKF-Schritt
    q = ekf.update(q, gyroV, accelV, magV, dt);

    // Quaternion -> Euler
    EulerDeg e = quatToEulerDeg(q);

    // x-Achse = Roll
    lastAngleDeg = e.roll;
    return lastAngleDeg;
}
