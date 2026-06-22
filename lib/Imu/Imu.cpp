#include "Imu.h"
#include <Wire.h>

bool Imu::begin() {
    // The onboard LSM6DS3TR-C is power-gated; enable it before touching the bus.
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
    delay(10);

    // Guard against a dead/shorted internal bus: the nRF52 Wire layer's TWIM wait
    // loops have no timeout, so a stuck-low line would hang begin_I2C() forever.
    // A healthy idle I2C bus rests high; bail out if either line reads low.
    pinMode(PIN_WIRE1_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE1_SCL, INPUT_PULLUP);
    delayMicroseconds(20);
    if (digitalRead(PIN_WIRE1_SDA) == LOW || digitalRead(PIN_WIRE1_SCL) == LOW) {
        return false;                                      // bus stuck low — skip
    }

    Wire1.begin();                                          // internal bus: SDA17/SCL16
    if (!dev_.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire1)) {  // 0x6A
        return false;
    }

    dev_.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
    dev_.setAccelDataRate(LSM6DS_RATE_104_HZ);
    return true;
}

bool Imu::sample(ImuData& out) {
    unsigned long now = millis();
    if (now - lastSample_ < PERIOD_MS) {
        return false;
    }
    lastSample_ = now;

    sensors_event_t accel, gyro, temp;
    dev_.getEvent(&accel, &gyro, &temp);   // gyro/temp unused — accel only

    out.acc_x        = accel.acceleration.x;
    out.acc_y        = accel.acceleration.y;
    out.acc_z        = accel.acceleration.z;
    out.timestamp_ms = now;
    return true;
}
