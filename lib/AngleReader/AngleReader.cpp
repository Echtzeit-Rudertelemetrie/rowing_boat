// ─────────────────────────────────────────────────────────────────────────────
// AngleReader.cpp — Orientierung aus dem ICM-20948 (Accel + Gyro + AK09916-Mag)
// ─────────────────────────────────────────────────────────────────────────────
#include "AngleReader.h"

// ── (1) Magnetometer-Kalibrierung ─────────────────────────────────────────────

const float AngleReader::MAG_A[3][3] = {
    {1.97954f, 0.0f, 0.0f},
    {0.0f, 0.976826f, 0.0f},
    {0.0f, 0.0f, 0.517154f}
};


const float AngleReader::MAG_B[3] = {39.4022f, -96.3556f, -83.9676f};

// Eigene float-Konstante statt Arduinos DEG_TO_RAD: das Makro ist ein
// double-Literal und zieht die ganze Rechnung in Software-double — die
// S3-FPU kann nur single precision, double kostet ein Vielfaches.
static constexpr float kDegToRad = 0.017453293f;

static Quat makeIdentityQuat() {
    Quat q0;
    q0.m[0][0] = 1.0f;
    return q0;
}

// Numerisch stabile Online-Varianz (Welford). Nur von calibrateSensorNoise()
// genutzt; die Normierung mit (n-1) entspricht MATLABs var() in
// sensor_noise_calibration.m, damit beide Seiten denselben Wert liefern.
namespace {
struct Welford {
    float mean = 0.0f;
    float m2   = 0.0f;
    int   n    = 0;

    void add(float x) {
        ++n;
        const float delta = x - mean;
        mean += delta / static_cast<float>(n);
        m2   += delta * (x - mean);
    }

    float variance() const {
        if (n < 2) return 0.0f;
        const float v = m2 / static_cast<float>(n - 1);
        return (v > 0.0f) ? v : 0.0f;   // Rundung kann minimal unter 0 rutschen
    }
};
} // namespace

AngleReader::AngleReader()
    : ekf(kDefaultGyroNoise, kDefaultAccelNoise, kDefaultMagNoise),
      q(makeIdentityQuat()),
      initialized(false),
      hwOk(false),
      lastUpdateUs(0),
      lastAngleDeg(0.0f),
      gyroOffset{0.0f, 0.0f, 0.0f} {
    // KEIN Hardware-Zugriff im Konstruktor! Der laeuft als globale Static-Init
    // noch vor setup()/Serial.begin(). I2C-Init passiert in begin(). Der EKF
    // startet deshalb mit den kDefault*Noise-Werten; calibrateSensorNoise()
    // in begin() ersetzt sie bei plausibler Messung durch die gemessenen.
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
    calibrateSensorNoise();
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

// ── Sensor-Rauschen im Stillstand messen (Sensor muss dabei still liegen) ────
// Ersetzt bei plausiblem Ergebnis die Konstruktor-Defaults im EKF. Schlaegt
// die Messung fehl oder wirkt sie unplausibel (Sensor wurde bewegt), bleiben
// die kDefault*Noise-Werte aktiv.
void AngleReader::calibrateSensorNoise() {
    Serial.println("Rauschmessung (Sensor ruhig halten)...");

    // Akkumulatoren PRO ACHSE, auf den ROHEN Messwerten. Zwei Fallen stecken hier:
    //
    // 1) Die Achsen duerfen nicht in einen gemeinsamen Mittelwert wandern. In Ruhe
    //    liegt eine Accel-Achse bei ~g und die anderen bei ~0 — eine ueber alle drei
    //    gepoolte Varianz misst dann den Abstand ZWISCHEN den Achsen statt des
    //    Rauschens (beim normierten Vektor: 0.22 statt 1e-6).
    //
    // 2) Gemessen wird ROH, nicht normiert. Beim Normieren faellt das Rauschen
    //    entlang der dominanten Richtung heraus (aus (1+e0, e1, e2) wird nach der
    //    Division ~1.0), die Schwerkraft-Achse haette also scheinbar Varianz ~0.
    //    Welche Achse das trifft, haengt aber an der KALIBRIERLAGE — wir wuerden
    //    die Boot-Orientierung der Dolle in R einbetonieren, und sobald sich das
    //    Ruder dreht, passt R nicht mehr. Die Varianz der rohen Komponenten ist
    //    dagegen eine echte Sensoreigenschaft (MEMS-Achsen rauschen wirklich
    //    unterschiedlich) und dreht sich mit dem Body-Frame mit. Die Umrechnung in
    //    den normierten Messraum, in dem z/h leben, passiert unten ueber 1/|v|^2.
    //
    // Welford statt sum/sumSq: die Lehrbuchformel E[x^2]-E[x]^2 loescht sich in
    // float aus, wenn der Mittelwert gross gegen das Rauschen ist (Accel: ~9.81
    // vs ~0.009) — gemessen kamen dabei negative Varianzen heraus.
    Welford gyroAcc[3], accelAcc[3], magAcc[3];
    Welford accelNorm, magNorm;   // fuer die Skalierung roh -> normiert
    int collected = 0;
    const unsigned long deadlineMs = millis() + 3000; // Notausstieg, falls keine Daten kommen

    while (collected < NOISE_CALIB_SAMPLES && millis() < deadlineMs) {
        Vec3 gyroV, accelV, magV;
        if (readSample(gyroV, accelV, magV)) {
            float na = norm3(accelV);
            float nm = norm3(magV);
            if (na < 1e-9f || nm < 1e-9f) continue; // Nullvektor -> Sample verwerfen

            for (int i = 0; i < 3; ++i) {
                gyroAcc[i].add(gyroV.m[i][0]);
                accelAcc[i].add(accelV.m[i][0]);
                magAcc[i].add(magV.m[i][0]);
            }
            accelNorm.add(na);
            magNorm.add(nm);
            ++collected;
        }
        delay(1);
    }

    const int MIN_NOISE_SAMPLES = 50;
    if (collected < MIN_NOISE_SAMPLES) {
        Serial.println("Rauschmessung: zu wenig Samples, bleibe bei Default-Werten.");
        return;
    }

    // z und h im EKF sind normierte Vektoren -> R muss im normierten Raum leben.
    // Isotropes Rohrauschen sigma^2 wird durch die Normierung zu sigma^2/|v|^2.
    // Der Gyro geht roh in die Praediktion ein und bleibt daher unskaliert.
    const float accelScale = 1.0f / (accelNorm.mean * accelNorm.mean);
    const float magScale   = 1.0f / (magNorm.mean * magNorm.mean);

    Vec3 gyroVar, accelVar, magVar;
    for (int i = 0; i < 3; ++i) {
        gyroVar.m[i][0]  = gyroAcc[i].variance();
        accelVar.m[i][0] = accelAcc[i].variance() * accelScale;
        magVar.m[i][0]   = magAcc[i].variance()   * magScale;
    }

    for (int i = 0; i < 3; ++i) {
        if (gyroVar.m[i][0]  > kNoisePlausibilityLimit ||
            accelVar.m[i][0] > kNoisePlausibilityLimit ||
            magVar.m[i][0]   > kNoisePlausibilityLimit) {
            Serial.println("Rauschmessung: Sensor war nicht still, bleibe bei Default-Werten.");
            return;
        }
    }

    ekf.setNoise(gyroVar, accelVar, magVar);
    Serial.printf("Rauschwerte gemessen (%d Samples), Varianz je EKF-Achse:\n", collected);
    Serial.printf("  gyro  [(rad/s)^2]: %.3e %.3e %.3e\n",
                  gyroVar.m[0][0], gyroVar.m[1][0], gyroVar.m[2][0]);
    Serial.printf("  accel [normiert] : %.3e %.3e %.3e\n",
                  accelVar.m[0][0], accelVar.m[1][0], accelVar.m[2][0]);
    Serial.printf("  mag   [normiert] : %.3e %.3e %.3e\n",
                  magVar.m[0][0], magVar.m[1][0], magVar.m[2][0]);
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

        ekf.setReferences(q, accelV, magV);

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
