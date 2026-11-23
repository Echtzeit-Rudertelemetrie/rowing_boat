// Wegwerf-Diagnose: prueft beide Anschlusswege des GPS-Moduls gleichzeitig.
//   1) I2C/DDC auf 0x42 — FIFO-Fuellstand lesen und NMEA ausgeben
//   2) UART auf D10 — Leitungszustand und Mitschnitt bei 115200
// Laeuft parallel dazu ein Mitschnitt am USB-Port des Moduls, ist belegt, ob
// das Modul im selben Moment sendet.
#include <Arduino.h>
#include <Wire.h>

static const uint8_t UBLOX_ADDR = 0x42;
static const int GPS_RX = 9;   // D10 <- GPS TX

static float highFraction(int gpio) {
    pinMode(gpio, INPUT_PULLDOWN);
    delayMicroseconds(200);
    uint32_t high = 0, total = 0;
    const uint32_t start = millis();
    while (millis() - start < 150) { high += digitalRead(gpio) ? 1 : 0; total++; }
    pinMode(gpio, INPUT);
    return total ? (100.0f * high) / total : 0.0f;
}

// u-blox DDC: 0xFD/0xFE halten die Anzahl wartender Bytes, danach liefert der
// Datenstrom sie aus. Ein leerer FIFO antwortet mit 0xFF-Fuellbytes.
static uint16_t ddcPending() {
    Wire.beginTransmission(UBLOX_ADDR);
    Wire.write(0xFD);
    if (Wire.endTransmission(false) != 0) return 0xFFFF;
    if (Wire.requestFrom((int)UBLOX_ADDR, 2) != 2) return 0xFFFF;
    const uint8_t hi = Wire.read(), lo = Wire.read();
    return ((uint16_t)hi << 8) | lo;
}

static void ddcRead(uint16_t pending) {
    String text;
    uint16_t got = 0;
    while (got < pending && text.length() < 160) {
        const uint8_t chunk = min<uint16_t>(32, pending - got);
        if (Wire.requestFrom((int)UBLOX_ADDR, (int)chunk) != chunk) break;
        while (Wire.available()) {
            const char c = (char)Wire.read();
            got++;
            if (c != (char)0xFF && text.length() < 160 && (isPrintable(c) || c == '$')) text += c;
        }
    }
    Serial.printf("  gelesen %u B: %s\n", got, text.c_str());
}

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("\n=== Beide Wege pruefen ===");
}

void loop() {
    Serial.print("I2C-Scan: ");
    Wire.begin();
    bool found = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X%s ", a, a == UBLOX_ADDR ? "(u-blox)" : (a == 0x69 ? "(IMU)" : ""));
            if (a == UBLOX_ADDR) found = true;
        }
    }
    Serial.println();

    if (found) {
        const uint16_t pending = ddcPending();
        if (pending == 0xFFFF)      Serial.println("  DDC: Fuellstand nicht lesbar");
        else if (pending == 0)      Serial.println("  DDC: FIFO leer");
        else { Serial.printf("  DDC: %u B wartend\n", pending); ddcRead(pending); }
    } else {
        Serial.println("  0x42 antwortet nicht");
    }
    Wire.end();

    Serial.printf("D10: %.1f %% HIGH bei Pulldown\n", highFraction(GPS_RX));
    Serial2.begin(115200, SERIAL_8N1, GPS_RX, -1);
    String text;
    uint32_t n = 0;
    const uint32_t start = millis();
    while (millis() - start < 1200) {
        while (Serial2.available()) {
            const char c = (char)Serial2.read();
            n++;
            if (text.length() < 80 && isPrintable(c)) text += c;
        }
    }
    Serial2.end();
    Serial.printf("  UART @115200: %lu B  %s\n---\n", (unsigned long)n, text.c_str());
}
