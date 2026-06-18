#pragma once

#include <Arduino.h>

class ForceReader {
public:
    ForceReader();
    float sampleForce();

private:
    static constexpr unsigned long SAMPLE_INTERVAL_US = 10000UL; // 10 ms = 100 Hz
};