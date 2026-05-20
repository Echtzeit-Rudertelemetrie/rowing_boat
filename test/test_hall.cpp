#include <Wire.h>
#include <Arduino.h>

#define AS5600_ADDR        0x36
#define REG_ANGLE_H        0x0E
#define SAMPLE_INTERVAL_US 10000  // 100 Hz
#define EMA_ALPHA          0.1f

// ─────────────────────────────────────────────────────────────
uint32_t lastSample = 0;
float    offset     = 0.0f;
float    emaAngle   = 0.0f;
bool     emaInit    = false;

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
float applyEMA(float newVal) {
    if (!emaInit) {
        emaAngle = newVal;
        emaInit  = true;
        return newVal;
    }
    float delta = newVal - emaAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    emaAngle = wrapAngle(emaAngle + EMA_ALPHA * delta);
    return emaAngle;
}

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    delay(100); // AS5600 startup settling

    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    }
    offset = raw;
}

void printMagnitude() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x1B);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    uint16_t magnitude = (Wire.read() << 8) | Wire.read();
    magnitude &= 0x0FFF;
    Serial.printf(">magnitude:%d\n", magnitude);
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = micros();
    if (now - lastSample >= SAMPLE_INTERVAL_US) {
        lastSample = now;

        float raw = readRawAngle();
        if (raw < 0.0f) return;

        float angle    = wrapAngle(raw - offset);
        float filtered = applyEMA(angle);

        Serial.printf(">angle_raw:%.2f\n", angle);
        //Serial.printf(">angle_filtered:%.2f\n", filtered);

        printMagnitude();
    }
    delay(500);
}

/*

Auslesen des AGS Registers
#include <Wire.h>

#define AS5600_ADDR 0x36
#define AGC_REG     0x1A

void setup() {
  Serial.begin(115200);
  Wire.begin();

  uint8_t agc = readAGC();

  Serial.print("AGC = ");
  Serial.println(agc);
}

void loop() {
  uint8_t agc = readAGC();

  Serial.print("AGC = ");
  Serial.println(agc);

  delay(500);
}

uint8_t readAGC() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AGC_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}
*/