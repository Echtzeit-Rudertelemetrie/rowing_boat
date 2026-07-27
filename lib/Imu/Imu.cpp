#include "Imu.h"

#include <Wire.h>

namespace {
constexpr float kMgToMs2 = 9.80665f / 1000.0f;
constexpr bool kAd0High = true;
constexpr uint8_t kInitAttempts = 5;
}

bool Imu::begin() {
    Wire.begin();

    for (uint8_t attempt = 0; attempt < kInitAttempts; ++attempt) {
        if (sensor_.begin(Wire, kAd0High) == ICM_20948_Stat_Ok) {
            configure();
            available_ = true;
            latest_ = read();
            return true;
        }
        delay(100);
    }

    available_ = false;
    return false;
}

void Imu::configure() {
    ICM_20948_fss_t fullScale{};
    fullScale.a = gpm4;
    fullScale.g = dps500;
    sensor_.setFullScale(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
        fullScale
    );

    ICM_20948_dlpcfg_t lowPass{};
    lowPass.a = acc_d473bw_n499bw;
    lowPass.g = gyr_d361bw4_n376bw5;
    sensor_.setDLPFcfg(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr,
        lowPass
    );
    sensor_.enableDLPF(ICM_20948_Internal_Acc, true);
}

ImuData Imu::read() {
    if (!available_ || !sensor_.dataReady()) return latest_;

    sensor_.getAGMT();
    latest_.acc_x = sensor_.accX() * kMgToMs2;
    latest_.acc_y = sensor_.accY() * kMgToMs2;
    latest_.acc_z = sensor_.accZ() * kMgToMs2;
    latest_.timestamp_ms = millis();
    return latest_;
}
