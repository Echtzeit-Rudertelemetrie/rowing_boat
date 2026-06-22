#pragma once
#include <Arduino.h>
#include <Adafruit_LSM6DS3TRC.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU driver — the XIAO nRF52840 Sense's ONBOARD 6-axis LSM6DS3TR-C.
// It sits on the internal I2C bus Wire1 (SDA=17 / SCL=16, address 0x6A) and must
// be powered by driving PIN_LSM6DS3TR_C_POWER (pin 15) HIGH before use.
//
// We only need ACCELERATION for the telemetry packet; the gyro is left unused.
// ─────────────────────────────────────────────────────────────────────────────

// Compact IMU payload for the BLE TelemetryPacket (16 bytes, packed).
struct __attribute__((packed)) ImuData {
    float    acc_x;          // [m/s^2]
    float    acc_y;
    float    acc_z;
    uint32_t timestamp_ms;   // millis() when sampled
};
static_assert(sizeof(ImuData) == 16, "ImuData must stay 16 bytes (wire format)");

class Imu {
public:
    bool begin();                 // power on, Wire1, init LSM6DS3TR-C; false if absent
    bool sample(ImuData& out);    // rate-limited (~60 Hz); true when a new sample is written

private:
    static constexpr unsigned long PERIOD_MS = 16;   // ~60 Hz

    Adafruit_LSM6DS3TRC dev_;
    unsigned long       lastSample_ = 0;
};
