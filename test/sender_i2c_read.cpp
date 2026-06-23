#include <Arduino.h>
#include <Wire.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    delay(100);

    Wire.begin();  // SDA/SCL default pins für ESP32-S3: GPIO8/GPIO9

    Serial.println("=== I2C Scan ===");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X\n", addr);
            found++;
        }
    }
    Serial.printf("================\n%u device(s) found\n", found);
}

void loop() {}