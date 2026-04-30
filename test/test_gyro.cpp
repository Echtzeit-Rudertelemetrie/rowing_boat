#include <Arduino.h>
#include <ESP32Servo.h>
#include "gyro.h"
#include "../lib/logging.hpp"
#include <LittleFS.h>

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
const float ANGLE_FINISH_DEG       = 0.0f;
const float ANGLE_CATCH_DEG        = 90.0f;
const unsigned long RECOVERY_DURATION_MS = 2000;
const unsigned long DRIVE_DURATION_MS    = 1000;

Servo rowingServo;
float currentAngleDeg = ANGLE_FINISH_DEG;

float easeInOut(float t) {
    return (1.0f - cosf(t * PI)) * 0.5f;
}

void recoveryPhase() {
    unsigned long start = millis();
    while (millis() - start < RECOVERY_DURATION_MS) {
        float t = (float)(millis() - start) / (float)RECOVERY_DURATION_MS;
        currentAngleDeg = ANGLE_FINISH_DEG + t * (ANGLE_CATCH_DEG - ANGLE_FINISH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot(currentAngleDeg);
    }
}

void drivePhase() {
    unsigned long start = millis();
    while (millis() - start < DRIVE_DURATION_MS) {
        float t = (float)(millis() - start) / (float)DRIVE_DURATION_MS;
        float progress = easeInOut(t);
        currentAngleDeg = ANGLE_CATCH_DEG + progress * (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG);
        rowingServo.write((int)currentAngleDeg);
        sampleAndPlot(currentAngleDeg);
    }
}

void setup() {
    Serial.begin(115200);
   // LittleFS.begin();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    rowingServo.setPeriodHertz(50);
    rowingServo.attach(SERVO_PIN);
    rowingServo.write((int)ANGLE_FINISH_DEG);
    delay(500);

    setupGyro();
    calibrateGyro();
    delay(100);
}

void loop() {
    recoveryPhase();
    drivePhase();
}