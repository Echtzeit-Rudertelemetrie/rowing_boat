#include "Imu.h"
#include "BoatImuConfig.h"
#include <Wire.h>
#include <math.h>

namespace {
int16_t roundedInt16(float value) {
    if (!isfinite(value)) return 0;
    value = fminf(32767.0f, fmaxf(-32768.0f, value));
    return static_cast<int16_t>(lroundf(value));
}
}

Imu::Imu() : angleReader_(BoatImuConfig::MAG_CALIBRATION) {}

bool Imu::begin() {
    Wire.begin();
    Wire.setClock(400000);
    available_ = angleReader_.begin();
    return available_;
}

void Imu::update() {
    if (!available_) return;
    angleReader_.sampleAndCalculateAngle();
    const AngleDiagnostics& d = angleReader_.diagnostics();
    latest_.acc_x_mg = roundedInt16(d.accel[0]);
    latest_.acc_y_mg = roundedInt16(d.accel[1]);
    latest_.acc_z_mg = roundedInt16(d.accel[2]);
    latest_.roll_cdeg = roundedInt16(d.rollDeg * 100.0f);
    latest_.pitch_cdeg = roundedInt16(d.pitchDeg * 100.0f);
    latest_.yaw_cdeg = roundedInt16(d.yawDeg * 100.0f);
    latest_.timestamp_ms = millis();
}
