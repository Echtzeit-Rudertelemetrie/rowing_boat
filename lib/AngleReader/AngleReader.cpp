// ─────────────────────────────────────────────────────────────────────────────
// AngleReader.cpp
// ─────────────────────────────────────────────────────────────────────────────
#include "AngleReader.h"
#include "gyro.h"
#include <Adafruit_MMC56x3.h>
#include <Adafruit_MPU6050.h>

// vorhandene Sensor-Objekte
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
Adafruit_MPU6050 mpu;
float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

// ── (1) Magnetometer-Kalibrierung – aus magcal in MATLAB übernommen ───────────
// TODO: Kalibrierungswerte irgendwie dynamisch bekommen. Sind das überhaupt Kalibrierungswerte?
const float AngleReader::MAG_A[3][3] = {
    {1.0962f, 0.0f,    0.0f   },
    {0.0f,    1.4670f, 0.0f   },
    {0.0f,    0.0f,    0.6219f}
};

const float AngleReader::MAG_B[3] = {-115.4853f, 6.6805f, -14.4159f}; //TODO: Wie bekommen wir die Kalibrierungswerte dymanisch?

static Quat makeIdentityQuat() {
    Quat q0;
    q0.m[0][0] = 1.0f;
    return q0;
}

AngleReader::AngleReader()
    : ekf(0.09f /*gyro*/, 0.25f /*accel*/, 0.64f /*mag*/),
      q(makeIdentityQuat()),
      initialized(false),
      lastUpdateUs(0) {
    // KEIN Hardware-Zugriff im Konstruktor! Der laeuft als globale Static-Init
    // noch vor setup()/Serial.begin(). I2C-Init passiert in begin().
}

bool AngleReader::begin() {
    bool ok = true;

    if (!mpu.begin(SDA, SCL))
    {
        Serial.println("Failed to find MPU6050 chip");
        ok = false;
    }
    else
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // ── MMC56X3 Init ──────────────────────────────────────────────────────────
    if (!mmc.begin(SDA, SCL))
    {
        Serial.println("Failed to find MMC56X3 chip");
        ok = false;
    }
    else
    {
        mmc.setDataRate(1000000UL / SAMPLE_INTERVAL_US);
    }

    return ok;
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

// ── Hauptfunktion ─────────────────────────────────────────────────────────────
float AngleReader::sampleAndCalculateAngle() {
    unsigned long nowUs = micros();

    // 1) Einmalige Initialisierung mit erster Messung
    if (!initialized) {
        sensors_event_t a, g, temp, m;
        mpu.getEvent(&a, &g, &temp);
        mmc.getEvent(&m);

        float gx = g.gyro.x - gyroOffsetX;
        float gy = g.gyro.y - gyroOffsetY;
        float gz = g.gyro.z - gyroOffsetZ;

        float magRaw[3] = {m.magnetic.x, m.magnetic.y, m.magnetic.z};
        float magCal[3];
        calibrateMag(magRaw, magCal);

        Vec3 gyroV  = vec3(gz, gy, gx);
        Vec3 accelV = vec3(a.acceleration.z, a.acceleration.y, a.acceleration.x);
        Vec3 magV   = vec3(magCal[0], magCal[1], magCal[2]);

        ekf.setReferences(accelV, magV);

        lastUpdateUs = nowUs;
        initialized = true;
        return 0.0f;
    }

    // 2) Wirklich nur alle 10 ms weiterrechnen //TODO: Führt das zu ungewollten aussetzern?
    if ((nowUs - lastUpdateUs) < SAMPLE_INTERVAL_US) {
        return 0.0f;
    }

    float dt = (nowUs - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = nowUs;

    // Sensoren lesen
    sensors_event_t a, g, temp, m;
    mpu.getEvent(&a, &g, &temp);
    mmc.getEvent(&m);

    // Gyro-Offset abziehen
    float gx = g.gyro.x - gyroOffsetX;
    float gy = g.gyro.y - gyroOffsetY;
    float gz = g.gyro.z - gyroOffsetZ;

    // Magnetometer kalibrieren
    float magRaw[3] = {m.magnetic.x, m.magnetic.y, m.magnetic.z};
    float magCal[3];
    calibrateMag(magRaw, magCal);

    // Achsen-Mapping
    Vec3 gyroV  = vec3(gz, gy, gx);
    Vec3 accelV = vec3(a.acceleration.z, a.acceleration.y, a.acceleration.x);
    Vec3 magV   = vec3(magCal[0], magCal[1], magCal[2]);

    // EKF-Schritt
    q = ekf.update(q, gyroV, accelV, magV, dt);

    // Quaternion -> Euler
    EulerDeg e = quatToEulerDeg(q);

    // x-Achse = Roll
    return e.roll;
}