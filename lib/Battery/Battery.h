#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Battery monitor — XIAO nRF52840 onboard LiPo charging circuit (BQ25101).
//
// What the board exposes (Seeed_XIAO_nRF52840_Sense variant):
//   VBAT_ENABLE (14)          drive LOW to connect the battery voltage divider
//   PIN_VBAT (32)             ADC input after the on-board 1 MΩ / 0.51 MΩ divider
//   PIN_CHARGING_CURRENT (22) OUTPUT: HIGH = default (slow), LOW = fast charge
//
// There is NO GPIO that reports charge status (the BQ25101 ~CHG pin only drives
// the orange LED). We approximate "charging" from USB-power presence via the
// nRF52 VBUS detector (NRF_POWER->USBREGSTATUS): if USB 5V is present the charger
// is active. Guarded with #ifdef so the file still compiles on a future board.
// ─────────────────────────────────────────────────────────────────────────────

// Compact battery payload for the BLE TelemetryPacket (4 bytes, packed).
struct __attribute__((packed)) BatteryData {
    uint16_t millivolts;   // battery voltage [mV]
    uint8_t  percent;      // rough state-of-charge estimate 0..100
    uint8_t  charging;     // 1 = USB power present (charging/topped off), else 0
};
static_assert(sizeof(BatteryData) == 4, "BatteryData must stay 4 bytes (wire format)");

class Battery {
public:
    void        begin();   // configure ADC reference + battery pins
    BatteryData read();     // sample voltage, estimate %, detect USB power

private:
    // On-board divider ratio: (1 MΩ + 0.51 MΩ) / 0.51 MΩ ≈ 2.9608
    static constexpr float    DIVIDER_RATIO = (1000.0f + 510.0f) / 510.0f;
    static constexpr float    ADC_REF_MV    = 3000.0f;   // AR_INTERNAL_3_0
    static constexpr float    ADC_MAX       = 4096.0f;   // 12-bit resolution
    static constexpr uint16_t BAT_EMPTY_MV  = 3300;      // ~0 %
    static constexpr uint16_t BAT_FULL_MV   = 3700;      // ~100 %
};
