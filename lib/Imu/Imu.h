#pragma once
#include <Arduino.h>
#include "AngleReader.h"

// ─────────────────────────────────────────────────────────────────────────────
// IMU payload for the boat telemetry packet (id 0), laid into the angle region
// of the shared MeasurementPack. 16 bytes, packed, little-endian — must match
// the phone app's ImuSample decoder. Acceleration is stored in mg and Euler
// angles in centidegrees so acceleration, full orientation and timestamp fit.
//
// ─────────────────────────────────────────────────────────────────────────────
struct __attribute__((packed)) ImuData {
    int16_t  acc_x_mg;
    int16_t  acc_y_mg;
    int16_t  acc_z_mg;
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    uint32_t timestamp_ms;   // millis() when sampled
};
static_assert(sizeof(ImuData) == 16, "ImuData must stay 16 bytes (wire format)");

class Imu {
public:
    Imu();
    bool begin();
    void update();
    ImuData data() const { return latest_; }
    bool available() const { return available_; }
    const AngleDiagnostics& diagnostics() const { return angleReader_.diagnostics(); }

private:
    AngleReader angleReader_;
    ImuData latest_{};
    bool available_ = false;
};
