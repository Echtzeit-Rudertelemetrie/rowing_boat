#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// IMU — onboard LSM6DS3TR-C on the XIAO nRF52840 (Sense) telemetry hub.
// Reads acceleration over the internal I2C bus (Wire1, address 0x6A) via the
// Adafruit LSM6DS driver; the IMU is powered up in setupIMU() through
// PIN_LSM6DS3TR_C_POWER. The external GPS uses the primary Wire (D4/D5).
// (Gyro unused for now.)
// ─────────────────────────────────────────────────────────────

// Latest sample, packed for embedding into the BLE TelemetryPacket.
struct __attribute__((packed)) ImuData {
    float    acc_x, acc_y, acc_z;  // acceleration [m/s^2]
    uint32_t timestamp_ms;         // millis() when the sample was taken
};                                 // 3*4 + 4 = 16 bytes

// Initialize the MPU6050 (range + filter config).
// Returns false if the chip is not found; caller may continue without IMU.
bool setupIMU();

// Take a sample, rate-limited to ~60 Hz internally. Fills `out` and returns
// true when a fresh sample was taken; returns false (and leaves `out`
// untouched) when called before the next sample is due.
bool sampleIMU(ImuData& out);
