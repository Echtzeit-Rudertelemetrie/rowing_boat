#include "gyro.h"
#include <LittleFS.h>
#include "../logging.hpp"
#include <MadgwickAHRS.h>
#include <math.h>

#include "filter/kalman.hpp"
#include <Adafruit_MMC56x3.h> // MMC56X3 Magnetometer

// ─────────────────────────────────────────────────────────────────────────────
// Private Variables (Only visible inside gyro.cpp)
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu; //um mpu anzusprechen
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345); // unique sensor ID für das magnetometer
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
const unsigned long SAMPLE_INTERVAL_MS = 10; //alle 10 Millisekunden ausgeben

// Magnetometer aktiv → EKF bekommt absolute Yaw-Referenz, reduziert Z-Drift auf Roll
EKF ekf(true, 100.0f);                                  // Magnetometer, 100 Hz
static Vec4f ekf_quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // Initialquaternion (aktuelle Ausrichtung -> keine Drehung)


//setzt Gyro, Magnetometer und anderes auf
void setupGyro()
{
    LittleFS.begin(); //Flash Dateisystem starten

    if (!mpu.begin()) //checkt ob Sensor da ist, ansonsten streikt er
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
            delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //misst Beschleunigungen bis 8-fache Erdbeschleunigung
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); //misst Drehgeschwindigkeiten bis 500 Grad pro Sekunde
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); //Filterbandbreite 21 Hz: gleicht etwas Rauschen aus, reagiert aber nicht zu träge

    // ── MMC56X3 (magnetometer) Init, sonst streiken ──────────────────────────────────────────────────────────
    if (!mmc.begin())
    {
        Serial.println("Failed to find MMC56X3 chip");
        while (1)
            delay(10);
    }
    mmc.setDataRate(1000 / SAMPLE_INTERVAL_MS); // 100 Hz Abtastrate— passend zu SAMPLE_INTERVAL_MS
}

//Misst im Ruhezustand -> offsets und so
void calibrateGyro(int samples)
{
    sensors_event_t a, g, temp; //a = Accelerometer, g = Gyro, temp = temperatur
    float sumX = 0, sumY = 0, sumZ = 0; //Gyroachsen einzeln aufaddiert

    Serial.println("Calibrating Gyro...");
    for (int i = 0; i < samples; i++) //Gyro über 2 Sekunden 2000 mal auslesen für offset
    {
        mpu.getEvent(&a, &g, &temp);
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(1);
    }
    //Durchschnitt der Messung berechnen und offset festlegen
    gyroOffsetX = sumX / samples; 
    gyroOffsetY = sumY / samples;
    gyroOffsetZ = sumZ / samples;

    Serial.println("Calibration complete.");
}

static unsigned long lastSample = 0; //Zeit des letzten Serial-Plots
static unsigned long lastGyroUpdate = 0; //Zeit des letzten Sensorreads
static float angle = 0.0f; //integrierter Rohwinkel aus dem Gyro
static bool firstRun = true; //verhindert einen riesigen dt beim ersten Aufruf (weil integrieren und so)
static float ekf_angle_offset = 0.0f;   // EKF null point captured at startup
static bool ekf_initialized = false; //prüft, ob EKF-Zero schon gesetzt wurde
static bool mag_ref_set = false; //prüft, ob Magnetometer-Referenz schon gesetzt wurde

void sampleAndPlot(float servoAngle)
{

    unsigned long now = micros();

    // prevent massive dt spike on the very first run
    if (firstRun)
    {
        lastGyroUpdate = now;
        lastSample = millis(); //maybe lieber mikros?
        firstRun = false;
        return;
    }

    // 1. Calculate dt
    float dt = (now - lastGyroUpdate) / 1000000.0f; // µs → s , weil die Funktion, über die man integriert mit sekunden arbeitet
    lastGyroUpdate = now;

    // 2. Read sensors
    sensors_event_t a, g, temp, mag;
    mpu.getEvent(&a, &g, &temp);
    mmc.getEvent(&mag);


    // 3. Subtract gyro offset, integrate raw angle
    g.gyro.x -= gyroOffsetX;
    g.gyro.y -= gyroOffsetY;
    g.gyro.z -= gyroOffsetZ;

    angle -= g.gyro.x * dt; //nur den x Achsen Wert nach x integrieren

    // 4. Accelerometer angles per axis [rad]
    float accel_angle_x = atan2f(a.acceleration.y, a.acceleration.z); // rotation around X -> Brauchts das? Ist das korrekt?
    float accel_angle_y = atan2f(a.acceleration.x, a.acceleration.z); // rotation around Y -> Brauchts das? Ist das korrekt?
    float accel_angle_z = atan2f(a.acceleration.x, a.acceleration.y); // rotation around Z -> Brauchts das? Ist das korrekt?

    //Beim ersten magnetometer Wert eine Referenz setzen
    if (!mag_ref_set)
    {
        Vec3f mag_init = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
        ekf.setMagnetReference(mag_init);
        mag_ref_set = true;
    }

    // EKF update — magnetoVec jetzt mit echten Messwerten befüllt
    // Die Vektoren für das EKF werden hier so initialisiert, wie wir sie brauchen. Achsen werden so umgeordnet wie gebraucht (achtung bei Veränderungen)
    Vec3f gyroVec = {g.gyro.z, g.gyro.y, g.gyro.x};
    Vec3f accelVec = {a.acceleration.z, a.acceleration.y, a.acceleration.x};
    Vec3f magnetoVec = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

    //Der EKF kombiniert:

    //Gyro: schnelle Änderungen
    //Accelerometer: Richtung der Schwerkraft
    //Magnetometer: absolute Nord-/Yaw-Referenz
    //Ergebnis:
    //Eine neue Orientierungs-Quaternion.
    ekf_quaternion = ekf.update(ekf_quaternion, gyroVec, accelVec, magnetoVec, dt);

    // Roll aus Quaternion extrahieren [rad]. Also der Roll der verschiedenen Achsen? (aber was ist die w-Achse?)
    float qw = ekf_quaternion(0), qx = ekf_quaternion(1);
    float qy = ekf_quaternion(2), qz = ekf_quaternion(3);

    //Diese Formel ist laut Dominik (siehe Obsidian) und KI falsch 
    // macht gerade ?
    float angle_ekf = -atan2f(
        2.0f * (qw*qz + qx*qy),
        1.0f - 2.0f * (qy*qy + qz*qz));

    // 5. EKF Nullpunkt beim ersten gültigen Sample setzen — gleiche Logik wie angle_raw
    if (!ekf_initialized)
    {
        ekf_angle_offset = angle_ekf;
        ekf_initialized = true;
        return;
    }

    float angle_ekf_zeroed = angle_ekf - ekf_angle_offset;


    // 7. Print data only every SAMPLE_INTERVAL_MS (100 Hz)
    unsigned long nowMs = millis();
    if (nowMs - lastSample >= SAMPLE_INTERVAL_MS)
    {
        lastSample = nowMs;


    float angle_normed = angle * (180.0f / PI);
    float angle_EKF_normed = angle_ekf_zeroed * (180.0f / PI);
    // float angle_EKF_ekf_quaternion = ekf_quaternion * (180.0f / PI);
    float accel_x_normed = accel_angle_x * (180.0f / PI);
    float accel_y_normed = accel_angle_y * (180.0f / PI);
    float accel_z_normed = accel_angle_z * (180.0f / PI);

        Serial.printf(">imu/angle_EKF: %.4f\n", angle_EKF_normed);
        Serial.printf(">imu/angle_raw: %.4f\n", angle_normed);
        Serial.printf(">imu/Gyro_x: %.4f\n", g.gyro.x);
        Serial.printf(">imu/Gyro_y: %.4f\n", g.gyro.y);
        Serial.printf(">imu/Gyro_z: %.4f\n", g.gyro.z);

        // Serial.printf(">imu/angle_kalman: %.4f\n",  angle_EKF_ekf_quaternion);
        Serial.printf(">imu/accel_angle: %.4f\n", accel_y_normed);
        Serial.printf(">imu/Acce_x: %.4f\n", a.acceleration.x);
        Serial.printf(">imu/Acce_y: %.4f\n", a.acceleration.y);
        Serial.printf(">imu/Acce_z: %.4f\n", a.acceleration.z);
        // Magnetometer Rohwerte zur Diagnose
        Serial.printf(">mag/x: %.4f\n", mag.magnetic.x);
        Serial.printf(">mag/y: %.4f\n", mag.magnetic.y);
        Serial.printf(">mag/z: %.4f\n", mag.magnetic.z);
    }
}