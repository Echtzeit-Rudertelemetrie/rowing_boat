#pragma once
#include <Arduino.h>
#include <ICM_20948.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU payload for the boat telemetry packet (id 0), laid into the angle region
// of the shared MeasurementPack. 16 bytes, packed, little-endian — must match
// the phone app's ImuSample decoder (acc_x/y/z as float32, timestamp as uint32).
//
// ─────────────────────────────────────────────────────────────────────────────
struct __attribute__((packed)) ImuData {
    float    acc_x;          // [m/s^2]
    float    acc_y;
    float    acc_z;
    uint32_t timestamp_ms;   // millis() when sampled
};
static_assert(sizeof(ImuData) == 16, "ImuData must stay 16 bytes (wire format)");

class Imu {
public:
    bool begin();
    ImuData read();
    bool available() const { return available_; }

private:
    void configure();

    ICM_20948_I2C sensor_;
    ImuData latest_{};
    bool available_ = false;
};
