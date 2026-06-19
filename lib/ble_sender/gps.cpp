#include "gps.h"

#include <Wire.h>
#include <TinyGPSPlus.h>

namespace {
    constexpr uint8_t GPS_I2C_ADDR   = 0x42;  // u-blox DDC default address
    constexpr uint8_t GPS_READ_CHUNK = 32;    // bytes per I2C request

    TinyGPSPlus gps;
}

void setupGPS() {
    Wire.begin();
}

// u-blox DDC: requesting bytes returns the next part of the NMEA/UBX
// stream; a returned 0xFF means "no data available" and is skipped.
void updateGPS() {
    Wire.requestFrom(GPS_I2C_ADDR, GPS_READ_CHUNK);
    while (Wire.available()) {
        char c = Wire.read();
        if (c != (char)0xFF) {
            gps.encode(c);
        }
    }
}

GpsData getGpsData() {
    GpsData d;
    d.lat        = static_cast<int32_t>(gps.location.lat() * 1e6);
    d.lon        = static_cast<int32_t>(gps.location.lng() * 1e6);
    d.speed_mps  = gps.speed.mps();
    d.course_deg = gps.course.deg();
    d.satellites = static_cast<uint8_t>(gps.satellites.value());
    d.valid      = gps.location.isValid();
    return d;
}
