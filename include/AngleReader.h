#pragma once

#include <Arduino.h>
#include "orientation_ekf.h"

class AngleReader {
public:
    AngleReader();
    float sampleAndCalculateAngle();

private:
    static constexpr unsigned long SAMPLE_INTERVAL_US = 10000UL; // 10 ms = 100 Hz

    static const float MAG_A[3][3];
    static const float MAG_B[3];

    static void calibrateMag(const float raw[3], float out[3]);

    OrientationEKF ekf;
    Quat q;
    bool initialized;
    unsigned long lastUpdateUs;
};