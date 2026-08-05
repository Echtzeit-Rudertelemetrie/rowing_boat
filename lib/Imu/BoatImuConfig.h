#pragma once

#include "AngleReader.h"

namespace BoatImuConfig {

// These values belong only to the IMU installed in the boat. Replace them with
// the output of scripts/calibrate_magnetometer.py after calibrating the complete
// BLE hub in its final mounting position.
//
// Recalibrated 2026-07-29 from logs/boat_mag_02.csv: residual 4.59%, span ratio
// 0.659, field magnitude 40.2 uT. The soft-iron matrix reproduces the 2026-07-27
// run almost exactly (diagonal 1.284/1.463/0.927 then, 1.203/1.471/0.910 now) —
// two independent measurements agreeing on a property of the assembly, which is
// the best evidence available that both runs were clean. Hard iron did move
// though, from |b| 3.9 uT to 11.7 uT, which is what made the redo worthwhile.
//
// Previous values (2026-07-27, calibration/boat_mag_01):
// constexpr MagCalibration MAG_CALIBRATION = {
//     true,
//     {
//         {1.28429129f, -0.0190608838f, 0.103510479f},
//         {-0.0190608838f, 1.4633201f, 0.0141170056f},
//         {0.103510479f, 0.0141170056f, 0.927376894f},
//     },
//     {3.29506649f, -0.102700858f, 2.1655548f},
// };
constexpr MagCalibration MAG_CALIBRATION = {
    true,
    {
        {1.20315044f, -0.0354236374f, 0.109790254f},
        {-0.0354236374f, 1.47096324f, 0.074241181f},
        {0.109790254f, 0.074241181f, 0.909921722f},
    },
    {5.15215573f, 4.03903918f, -9.73854238f},
};

} // namespace BoatImuConfig
