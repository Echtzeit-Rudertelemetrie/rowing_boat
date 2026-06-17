#include "hall_angle.h"

// ─────────────────────────────────────────────────────────────
void HallAngle::begin() {
    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    }
    _offset      = raw;
    _lastSample  = micros();
}

// ─────────────────────────────────────────────────────────────
bool HallAngle::update() {
    uint32_t now = micros();
    if (now - _lastSample < SAMPLE_INTERVAL_US) return false;
    _lastSample = now;

    float raw = readRawAngle();
    if (raw < 0.0f) return false;

    raw = wrapAngle(raw - _offset);

    // Smallest angular difference (-180 .. +180)
    float diff = raw - _lastAngle;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (fabsf(diff) >= JUMP_THRESHOLD_DEG) {
        _calcMode = !_calcMode;
        Serial.println("Switching Calc Mode");
    }

    _lastAngle  = raw;
    _angleRaw   = raw;
    _angleCalc  = _calcMode ? (360.0f - raw) : raw;
    _agc        = readAGC();
    _magnitude  = readMagnitude();

    return true;
}

// ─────────────────────────────────────────────────────────────
float HallAngle::readRawAngle() const {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDR, 2);

    if (Wire.available() == 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        raw &= 0x0FFF;
        return raw * 360.0f / 4096.0f;
    }
    return -1.0f;
}

// ─────────────────────────────────────────────────────────────
uint8_t HallAngle::readAGC() const {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_AGC);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

// ─────────────────────────────────────────────────────────────
uint16_t HallAngle::readMagnitude() const {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_MAG);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDR, 2);
    uint16_t mag = (Wire.read() << 8) | Wire.read();
    return mag & 0x0FFF;
}

// ─────────────────────────────────────────────────────────────
float HallAngle::wrapAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}
