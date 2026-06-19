#include "imu.h"

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

namespace {
    constexpr unsigned long IMU_SAMPLE_INTERVAL_MS = 1000 / 60;  // ~60 Hz

    Adafruit_LSM6DS3TRC imu;
    unsigned long lastSample = 0;
}

bool setupIMU() {
    // The onboard LSM6DS3TR-C sits on the internal I2C bus (Wire1) and must be
    // powered via PIN_LSM6DS3TR_C_POWER before it answers on the bus.
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
    delay(10);  // let the IMU boot after power-on

    Wire1.begin();
    if (!imu.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire1)) {  // 0x6A on Wire1
        return false;
    }

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);  // ODR >= our ~60 Hz poll rate
    return true;
}

bool sampleIMU(ImuData& out) {
    unsigned long now = millis();
    if (now - lastSample < IMU_SAMPLE_INTERVAL_MS) {
        return false;
    }
    lastSample = now;

    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);  // driver fills all; we keep only accel

    out.acc_x = accel.acceleration.x;
    out.acc_y = accel.acceleration.y;
    out.acc_z = accel.acceleration.z;
    out.timestamp_ms = now;
    return true;
}
