#include "gyro.h"

// ─────────────────────────────────────────────────────────────────────────────
// Private Variables (Only visible inside gyro.cpp)
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
const unsigned long SAMPLE_INTERVAL_MS = 10;

// ─────────────────────────────────────────────────────────────────────────────
// Function Implementations
// ─────────────────────────────────────────────────────────────────────────────

void setupGyro()
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
            delay(10);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

void sampleAndPlot(float servoAngle)
{
    static unsigned long lastSample = 0;
    static unsigned long lastGyroUpdate = 0;
    static float winkel = 0.0f;
    static bool firstRun = true;

    unsigned long now = micros();

    // Prevent massive dt spike on the very first run
    if (firstRun)
    {
        lastGyroUpdate = now;
        lastSample = millis();
        firstRun = false;
    }

    // 1. Calculate dt
    float dt = (now - lastGyroUpdate) / 1000000.0f; // µs → s
    lastGyroUpdate = now;

    // 2. Read sensor and integrate EXACTLY ONCE per call
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    g.gyro.x -= gyroOffsetX;
    winkel += g.gyro.x * dt;

    // 3. Print data only every SAMPLE_INTERVAL_MS (100 Hz)
    unsigned long nowMs = millis();
    if (nowMs - lastSample >= SAMPLE_INTERVAL_MS)
    {
        lastSample = nowMs;

        // Serial.print(">");
        Serial.printf(">imu/Gyro_x: %.4f\n", g.gyro.x);
        Serial.printf(">imu/Winkel_x: %.4f\n", winkel * (180.0f / PI));
        Serial.printf(">imu/Acce_x: %.4f\n", a.acceleration.x);
        Serial.printf(">imu/Acce_y: %.4f\n", a.acceleration.y);
        Serial.printf(">imu/Acce_z: %.4f\n", a.acceleration.z);
        Serial.printf(">stroke/Servo_deg: %.1f\n", servoAngle);
        // Serial.println();
    }
}