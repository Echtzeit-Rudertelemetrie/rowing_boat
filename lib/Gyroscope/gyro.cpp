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
Adafruit_MPU6050 mpu;
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345); // unique sensor ID
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
const unsigned long SAMPLE_INTERVAL_MS = 10;

// Madgwick madgwick_filter;

struct __attribute__((packed)) Sample
{
    uint32_t ts;
    float gyro_x;
    float angle_raw;
    // float angle_madgwick;
    float angle_EKF;
    // float angle_mahony;
    float acc_x, acc_y, acc_z;
}; // 4 + 8*4 = 36 Bytes → 1MB ≈ 7 min @ 100Hz

// ─────────────────────────────────────────────────────────────────────────────
// 1D Kalman Filter State
// ─────────────────────────────────────────────────────────────────────────────
/*
struct KalmanFilter1D
{
    float angle = 0.0f; // estimated angle [rad]
    float bias = 0.0f;  // estimated gyro bias [rad/s]

    float P[2][2] = {{1, 0}, {0, 1}}; // error covariance matrix

    // tuning parameters
    float Q_angle = 0.001f; // process noise angle — lower = slower correction
    float Q_bias = 0.003f;  // process noise bias
    float R_meas = 0.01f;   // measurement noise accelerometer — higher = less accel influence

    float update(float gyro_rads, float accel_angle_rad, float dt)
    {
        // ── Predict ──────────────────────────────────────────────────────────
        float rate = gyro_rads - bias;
        angle += rate * dt;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // ── Update (accelerometer as measurement) ─────────────────────────────
        float S = P[0][0] + R_meas; // innovation covariance
        float K[2];
        K[0] = P[0][0] / S; // kalman gain for angle
        K[1] = P[1][0] / S; // kalman gain for bias

        float y = accel_angle_rad - angle; // innovation

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_tmp = P[0][0];
        float P01_tmp = P[0][1];
        P[0][0] -= K[0] * P00_tmp;
        P[0][1] -= K[0] * P01_tmp;
        P[1][0] -= K[1] * P00_tmp;
        P[1][1] -= K[1] * P01_tmp;

        return angle;
    }
};

KalmanFilter1D kalman;
*/

// Magnetometer aktiv → EKF bekommt absolute Yaw-Referenz, reduziert Z-Drift auf Roll
EKF ekf(true, 100.0f);                                  // Magnetometer, 100 Hz
static Vec4f ekf_quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // Initialquaternion

// ─────────────────────────────────────────────────────────────────────────────
// Function Implementations
// ─────────────────────────────────────────────────────────────────────────────

void setupGyro()
{
    LittleFS.begin();

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
            delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // ── MMC56X3 Init ──────────────────────────────────────────────────────────
    if (!mmc.begin())
    {
        Serial.println("Failed to find MMC56X3 chip");
        while (1)
            delay(10);
    }
    mmc.setDataRate(100); // 100 Hz — passend zu SAMPLE_INTERVAL_MS

    // madgwick_filter.begin(100);
    // startLogging("angle_log");
}

void calibrateGyro(int samples)
{
    sensors_event_t a, g, temp;
    float sumX = 0, sumY = 0, sumZ = 0;

    Serial.println("Calibrating Gyro...");
    for (int i = 0; i < samples; i++)
    {
        mpu.getEvent(&a, &g, &temp);
        sumX += g.gyro.x;
        sumY += g.gyro.y;
        sumZ += g.gyro.z;
        delay(1);
    }
    gyroOffsetX = sumX / samples;
    gyroOffsetY = sumY / samples;
    gyroOffsetZ = sumZ / samples;

    Serial.println("Calibration complete.");
}

static unsigned long lastSample = 0;
static unsigned long lastGyroUpdate = 0;
static float angle = 0.0f;
static bool firstRun = true;
static bool kalman_initialized = false;
static float accel_angle_offset = 0.0f; // mounting offset nulled at startup
static float ekf_angle_offset = 0.0f;   // EKF null point captured at startup
static bool ekf_initialized = false;
static bool mag_ref_set = false;

void sampleAndPlot(float servoAngle)
{

    unsigned long now = micros();

    // prevent massive dt spike on the very first run
    if (firstRun)
    {
        lastGyroUpdate = now;
        lastSample = millis();
        firstRun = false;
        return;
    }

    // 1. Calculate dt
    float dt = (now - lastGyroUpdate) / 1000000.0f; // µs → s
    lastGyroUpdate = now;

    // 2. Read sensors
    sensors_event_t a, g, temp, mag;
    mpu.getEvent(&a, &g, &temp);
    mmc.getEvent(&mag);


    // 3. Subtract gyro offset, integrate raw angle
    g.gyro.x -= gyroOffsetX;
    g.gyro.y -= gyroOffsetY;
    g.gyro.z -= gyroOffsetZ;

    angle -= g.gyro.x * dt;

    // 4. Accelerometer angles per axis [rad]
    float accel_angle_x = atan2f(a.acceleration.y, a.acceleration.z); // rotation around X
    float accel_angle_y = atan2f(a.acceleration.x, a.acceleration.z); // rotation around Y
    float accel_angle_z = atan2f(a.acceleration.x, a.acceleration.y); // rotation around Z


    if (!mag_ref_set)
    {
        Vec3f mag_init = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
        ekf.setMagnetReference(mag_init);
        mag_ref_set = true;
    }

    // EKF update — magnetoVec jetzt mit echten Messwerten befüllt
    Vec3f gyroVec = {g.gyro.z, g.gyro.y, g.gyro.x};
    Vec3f accelVec = {a.acceleration.z, a.acceleration.y, a.acceleration.x};
    Vec3f magnetoVec = {mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};

    // ── Offset-Kompensation ───────────────────────────────────────────────────
    // Radius offset: r_offset = Abstand Sensor → Drehpunkt [m], Richtung in Sensor-X (anpassen!)
    const float r_offset = 0.15f; // messen und eintragen

    // static float prev_omega_z = 0.0f;
    // float omega_z = g.gyro.z;                      // bereits offset-subtrahiert
    // float alpha_z = (omega_z - prev_omega_z) / dt; // Winkelbeschleunigung [rad/s²]
    // prev_omega_z = omega_z;

    // // Zentripetal- und Tangentialbeschleunigung im Sensor-Frame subtrahieren
    // // (Vorzeichen abhängig davon ob Sensor "vor" oder "hinter" dem Drehpunkt)
    // accelVec(0) -= omega_z * omega_z * r_offset; // zentripetal, entlang X
    // accelVec(1) -= alpha_z * r_offset;           // tangential, entlang Y

    ekf_quaternion = ekf.update(ekf_quaternion, gyroVec, accelVec, magnetoVec, dt);

    // Roll aus Quaternion extrahieren [rad]
    float qw = ekf_quaternion(0), qx = ekf_quaternion(1);
    float qy = ekf_quaternion(2), qz = ekf_quaternion(3);

    // float angle_ekf = atan2f(
    //     2.0f * (qw * qx + qy * qz),
    //     1.0f - 2.0f * (qx * qx + qy * qy));

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

        // float angle_madgwick = 0;
        // float angle_mahony   = 0;
        // Sample sample = {millis(), g.gyro.x, angle, angle_madgwick, angle_EKF, angle_mahony, g.acceleration.x, g.acceleration.y, g.acceleration.z};
        // writeSample(&sample);

        // madgwick_filter.updateIMU(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);
        // angle_madgwick = madgwick_filter.getRoll();

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
        // Serial.printf(">stroke/Servo_deg: %.1f\n", servoAngle);
        //  Serial.println();
    }
}
