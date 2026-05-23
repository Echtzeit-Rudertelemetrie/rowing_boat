#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"

#define AS5600_ADDR        0x36
#define REG_ANGLE_H        0x0E
#define AGC_REG            0x1A
#define MAG_REG            0x1B
#define SAMPLE_INTERVAL_US 10000  // 100 Hz

// ─────────────────────────────────────────────────────────────
uint32_t lastSample = 0;
float    offset     = 0.0f;
bool     calcMode   = false;
float    lastAngle  = 0.0f;

// ─────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────
float wrapAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// ─────────────────────────────────────────────────────────────
/*uint8_t readAGC() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AGC_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

// ─────────────────────────────────────────────────────────────
uint16_t readMagnitude() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(MAG_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    uint16_t magnitude = (Wire.read() << 8) | Wire.read();
    return magnitude & 0x0FFF;
} */

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    esp_bt_controller_deinit();
    Serial.println("Wi-Fi and Bluetooth are now disabled.");

    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    }
    offset = raw;
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = micros();
    if (now - lastSample >= SAMPLE_INTERVAL_US) {
        lastSample = now;

        float raw = readRawAngle();
        if (raw < 0.0f) return;

        raw = wrapAngle(raw - offset);

        // ── Teleplot output ───────────────────────────────────
        Serial.printf(">as5600/angle_raw: %.2f\n",       raw);
        //Serial.printf(">as5600/magnitude: %d\n",         readMagnitude());
        //Serial.printf(">as5600/agc: %d\n",               readAGC());
    }
    delay(5);
}