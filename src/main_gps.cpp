#include <Arduino.h>
#include "gps.h"

// ─────────────────────────────────────────────────────────────────────────────
// prod_gps: dedizierter GPS-ESP32 (board: nodemcu-32s)
// lib/GPS ist noch ein Stub — siehe TODOs in lib/GPS/gps.h
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== prod_gps: GPS ===");

    setupGPS();
}

void loop() {
    updateGPS();
}
