#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "gyro.h"

// ─────────────────────────────────────────────────────────────────────────────
// Board-Specific Configuration
// ─────────────────────────────────────────────────────────────────────────────
#ifdef ARDUINO_NodeMCU_32S
  const int SERVO_PIN = 13;
#elif defined(ARDUINO_ESP32_DEV)
  const int SERVO_PIN = 18;
#else
  const int SERVO_PIN = 13;
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Rowing stroke geometry & timing
// ─────────────────────────────────────────────────────────────────────────────
const float          ANGLE_FINISH_DEG      = 0.0f;
const float          ANGLE_CATCH_DEG       = 90.0f;
const unsigned long  RECOVERY_DURATION_MS  = 2000;
const unsigned long  DRIVE_DURATION_MS     = 1000;

Servo rowingServo;
float currentAngleDeg = ANGLE_FINISH_DEG;

// ─────────────────────────────────────────────────────────────────────────────
// AS5600 Hall Sensor
// ─────────────────────────────────────────────────────────────────────────────
#define AS5600_ADDR        0x36
#define REG_ANGLE_H        0x0E
#define SAMPLE_INTERVAL_US 10000   // 100 Hz

uint32_t lastHallSample = 0;
float    hallOffset     = 0.0f;

float readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() == 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        raw &= 0x0FFF;
        return raw * 360.0f / 4096.0f;
    }
    return -1.0f;
}

float wrapAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// Non-blocking hall sample — call as often as possible inside loops
void sampleHall() {
    uint32_t now = micros();
    if (now - lastHallSample >= SAMPLE_INTERVAL_US) {
        lastHallSample = now;
        float raw = readRawAngle();
        if (raw < 0.0f) return;
        raw = wrapAngle(raw - hallOffset);
        Serial.printf(">as5600/angle_raw: %.2f\n", raw);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Easing helper
// ─────────────────────────────────────────────────────────────────────────────
float easeInOut(float t) {
    return (1.0f - cosf(t * PI)) * 0.5f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rowing phases — both sensors sampled on every iteration
// ─────────────────────────────────────────────────────────────────────────────
void recoveryPhase() {
    unsigned long start = millis();
    while (millis() - start < RECOVERY_DURATION_MS) {
        float t = (float)(millis() - start) / (float)RECOVERY_DURATION_MS;
        currentAngleDeg = ANGLE_FINISH_DEG + t * (ANGLE_CATCH_DEG - ANGLE_FINISH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot(currentAngleDeg);   // gyro → Teleplot
        sampleHall();                     // AS5600 → Teleplot (rate-limited internally)
    }
}

void drivePhase() {
    unsigned long start = millis();
    while (millis() - start < DRIVE_DURATION_MS) {
        float t        = (float)(millis() - start) / (float)DRIVE_DURATION_MS;
        float progress = easeInOut(t);
        currentAngleDeg = ANGLE_CATCH_DEG + progress * (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot(currentAngleDeg);   // gyro → Teleplot
        sampleHall();                     // AS5600 → Teleplot (rate-limited internally)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Disable WiFi & Bluetooth (reduces noise on I2C / power rail)
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    esp_bt_controller_deinit();

    // I2C for AS5600
    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    } else {
        hallOffset = raw;   // zero the angle at startup
    }

    // Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    rowingServo.setPeriodHertz(50);
    rowingServo.attach(SERVO_PIN);
    rowingServo.write((int)ANGLE_FINISH_DEG);
    delay(500);

    // Gyro
    setupGyro();
    calibrateGyro();
    delay(100);
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    recoveryPhase();
    drivePhase();
}