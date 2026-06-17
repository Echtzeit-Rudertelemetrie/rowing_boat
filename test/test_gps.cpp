#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "gps.h"

// ─────────────────────────────────────────────────────────────────────────────
// Wiring
// ─────────────────────────────────────────────────────────────────────────────
// ELT0332 (NEO-M8T) – I2C: IO21 (SDA), IO22 (SCL)
// Adafruit 746 (MTK3339) – auskommentiert (kein Adapter verfügbar)

// HardwareSerial gpsSerial(2);
// TinyGPSPlus uartGps;
TinyGPSPlus i2cGps;

// ─────────────────────────────────────────────────────────────────────────────
// Setup / Loop
// ─────────────────────────────────────────────────────────────────────────────
void i2cScan() {
    Serial.println("=== I2C Scan ===");
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found: 0x%02X\n", addr);
            found = true;
        }
    }
    if (!found) Serial.println("  nothing found – check SDA/SCL wiring");
    Serial.println("================");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin();
    i2cScan();
    // gpsUartBegin(gpsSerial);
    gpsI2cBegin();
    gpsI2cSetRate(10);  // 10 Hz (Hardware-Maximum GPS+GLONASS: 5Hz, GPS-only: 10Hz)
    Serial.println("module,fix,lat,lon,speed_kmh,alt_m,sats,age_ms");
}

void loop() {
    // gpsUartUpdate(gpsSerial, uartGps);
    gpsI2cUpdate(i2cGps);

    static uint32_t last = 0;
    if (millis() - last < 100) return;  // 10 Hz (Hardware-Maximum NEO-M8T)
    last = millis();

    // Serial.printf("uart  | fix: %s | lat: %.6f | lon: %.6f | alt: %.1fm | speed: %.1fkm/h | sats: %lu\n",
    //     uartGps.location.isValid() ? "yes" : "no",
    //     uartGps.location.lat(), uartGps.location.lng(),
    //     uartGps.altitude.meters(), uartGps.speed.kmph(),
    //     uartGps.satellites.value());

    Serial.printf("i2c   | fix: %s | lat: %.6f | lon: %.6f | alt: %.1fm | speed: %.1fkm/h | sats: %lu | cog: %.1f\n",
        i2cGps.location.isValid() ? "yes" : "no",
        i2cGps.location.lat(), i2cGps.location.lng(),
        i2cGps.altitude.meters(), i2cGps.speed.kmph(),
        i2cGps.satellites.value(),
        i2cGps.course.deg());

    // Teleplot-Format für Matlab
    Serial.printf(">gps/fix: %d\n",       (int)i2cGps.location.isValid());
    Serial.printf(">gps/cog_deg: %.2f\n", i2cGps.course.deg());
    Serial.printf(">gps/speed_kmh: %.2f\n", i2cGps.speed.kmph());
    Serial.printf(">gps/sats: %lu\n",     i2cGps.satellites.value());
}
