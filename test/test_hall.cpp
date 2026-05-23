#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "hall_angle.h"

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
    Serial.println("Wi-Fi and Bluetooth disabled.");

    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    sensor.begin();
}

// ─────────────────────────────────────────────────────────────
void loop() {
    if (!sensor.update()) return;

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