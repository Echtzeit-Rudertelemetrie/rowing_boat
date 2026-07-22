#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU payload for the boat telemetry packet (id 0), laid into the angle region
// of the shared MeasurementPack. 16 bytes, packed, little-endian — must match
// the phone app's ImuSample decoder (acc_x/y/z as float32, timestamp as uint32).
//
// A central IMU is connected to the hub (the BLE sender). Values are simulated
// for now (see lib/SimData); a real LSM6DS / MPU driver can be added later and
// fill this same struct.
// ─────────────────────────────────────────────────────────────────────────────
struct __attribute__((packed)) ImuData {
    float    acc_x;          // [m/s^2]
    float    acc_y;
    float    acc_z;
    uint32_t timestamp_ms;   // millis() when sampled
};
static_assert(sizeof(ImuData) == 16, "ImuData must stay 16 bytes (wire format)");
