#pragma once

#include <Arduino.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────
// HallAngle
// AS5600 driver + offset zeroing + calcMode direction flip
// ─────────────────────────────────────────────────────────────
class HallAngle {
public:
    static constexpr uint32_t SAMPLE_INTERVAL_US  = 10000;  // 100 Hz
    static constexpr float    JUMP_THRESHOLD_DEG  = 10.0f;

    // Call once after Wire.begin(); zeros offset to current position
    void begin();

    // Call every loop iteration
    // Returns true when a new sample was taken, false otherwise
    bool update();

    float    getAngleRaw()  const { return _angleRaw;  }
    float    getAngleCalc() const { return _angleCalc; }
    uint8_t  getAGC()       const { return _agc;       }
    uint16_t getMagnitude() const { return _magnitude;  }

private:
    static constexpr uint8_t I2C_ADDR  = 0x36;
    static constexpr uint8_t REG_ANGLE = 0x0E;
    static constexpr uint8_t REG_AGC   = 0x1A;
    static constexpr uint8_t REG_MAG   = 0x1B;

    float    _offset     = 0.0f;
    float    _lastAngle  = 0.0f;
    bool     _calcMode   = false;
    uint32_t _lastSample = 0;

    float    _angleRaw   = 0.0f;
    float    _angleCalc  = 0.0f;
    uint8_t  _agc        = 0;
    uint16_t _magnitude  = 0;

    float    readRawAngle()  const;
    uint8_t  readAGC()       const;
    uint16_t readMagnitude() const;
    static float wrapAngle(float a);
};
