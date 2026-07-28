#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "AppTypes.h"

// ─────────────────────────────────────────────────────────────────────────────
// GPS driver — u-blox module on the primary I2C bus (DDC, "I2C-NMEA").
// XIAO ESP32S3 wiring: D4 = SDA (GPIO5), D5 = SCL (GPIO6), DDC address 0x42.
//
// The boat IMU shares this bus and is polled at ~100 Hz for the EKF, so the
// receiver must not hog it. update() therefore reads the DDC FIFO level first
// and fetches only the bytes actually waiting, instead of clocking out 32 bytes
// of 0xFF padding on every call, and it polls at a bounded interval rather than
// on every loop iteration.
//
// The receiver is configured for 10 Hz with GGA and RMC only (see
// scripts/configure_gps.py), which is about 1.5 kB/s — roughly 4 percent of the
// bus at 400 kHz.
// ─────────────────────────────────────────────────────────────────────────────

class Gps {
public:
    void    begin();      // claim the primary bus and probe the receiver
    void    update();     // drain the DDC FIFO into TinyGPSPlus (call often)
    GpsData data();       // latest decoded fix as GpsData

    bool     available()       const { return busOk_; }
    uint32_t charsProcessed()  const { return gps_.charsProcessed(); }
    uint32_t passedChecksums() const { return gps_.passedChecksum(); }
    uint32_t failedChecksums() const { return gps_.failedChecksum(); }
    uint32_t locationAgeMs()   const;

private:
    static constexpr uint8_t  I2C_ADDR     = 0x42;  // u-blox DDC address
    static constexpr uint8_t  LEN_REGISTER = 0xFD;  // FIFO level, big endian
    static constexpr uint8_t  READ_CHUNK   = 32;    // bytes per I2C transaction
    static constexpr uint16_t MAX_PER_POLL = 256;   // caps a single update()
    static constexpr unsigned long POLL_INTERVAL_MS = 20;
    static constexpr unsigned long PROBE_RETRY_MS   = 2000;

    bool     probe();     // does the receiver ACK its address?
    uint16_t pending();   // bytes waiting in the DDC FIFO

    bool          wireReady_ = false;  // bus usable at all
    bool          busOk_     = false;  // receiver answering
    unsigned long lastPoll_  = 0;
    unsigned long lastProbe_ = 0;
    TinyGPSPlus   gps_;
};
