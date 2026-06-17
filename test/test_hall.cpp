#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "hall_angle.h"

// ─────────────────────────────────────────────────────────────
static HallAngle sensor;

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

    Serial.printf(">as5600/angle_raw: %.2f\n",       sensor.getAngleRaw());
    Serial.printf(">as5600/angle_with_calc: %.2f\n", sensor.getAngleCalc());
    Serial.printf(">as5600/magnitude: %d\n",         sensor.getMagnitude());
    Serial.printf(">as5600/agc: %d\n",               sensor.getAGC());
}
