#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

// ─────────────────────────────────────────────────────────────────────────────
// GPS driver — external u-blox module on the primary I2C bus (DDC / "I2C-NMEA").
// XIAO nRF52840 wiring:  D4 -> SDA,  D5 -> SCL.  u-blox DDC slave address 0x42.
//
// The module streams NMEA over I2C; we poll it in update() and feed the bytes to
// TinyGPSPlus.  data() returns a compact, BLE-ready snapshot.
// ─────────────────────────────────────────────────────────────────────────────

// Compact GPS payload for the BLE TelemetryPacket (18 bytes, packed).
struct __attribute__((packed)) GpsData {
    int32_t lat;          // latitude  * 1e6  (degrees)
    int32_t lon;          // longitude * 1e6  (degrees)
    float   speed_mps;    // ground speed [m/s]
    float   course_deg;   // course over ground [deg]
    uint8_t satellites;   // satellites in use
    bool    valid;        // true once a position fix is available
};
static_assert(sizeof(GpsData) == 18, "GpsData must stay 18 bytes (wire format)");

class Gps {
public:
    void    begin();      // Wire.begin() on the primary bus
    void    update();     // poll the DDC FIFO and feed TinyGPSPlus (call often)
    GpsData data();       // latest decoded fix as GpsData

private:
    static constexpr uint8_t I2C_ADDR   = 0x42;  // u-blox DDC address
    static constexpr uint8_t READ_CHUNK = 32;    // bytes per I2C poll

    bool        busOk_ = false;  // false if the I2C lines aren't idle-high (stuck/shorted)
    TinyGPSPlus gps_;
};
