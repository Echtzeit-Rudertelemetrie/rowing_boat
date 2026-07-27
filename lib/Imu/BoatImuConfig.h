#pragma once

#include "AngleReader.h"

namespace BoatImuConfig {

// These values belong only to the IMU installed in the boat. Replace them with
// the output of scripts/calibrate_magnetometer.py after calibrating the complete
// BLE hub in its final mounting position.
constexpr MagCalibration MAG_CALIBRATION = {
    true,
    {
        {1.28429129f, -0.0190608838f, 0.103510479f},
        {-0.0190608838f, 1.4633201f, 0.0141170056f},
        {0.103510479f, 0.0141170056f, 0.927376894f},
    },
    {3.29506649f, -0.102700858f, 2.1655548f},
};

} // namespace BoatImuConfig
