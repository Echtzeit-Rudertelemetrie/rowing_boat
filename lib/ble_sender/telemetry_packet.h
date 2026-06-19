#pragma once

#include <Arduino.h>
#include "gps.h"
#include "imu.h"

// ─────────────────────────────────────────────────────────────
// TelemetryPacket — the onboard-sensor packet broadcast to phones over BLE
// (one notification per packet), assembled at ~3.5 Hz from the latest GPS
// fix and IMU sample.
//
// Oarlock data is NOT carried here: RowingPacket (lib/ESPNOW/espnow.h,
// 133 bytes) arrives over UART from the aggregator and is forwarded as-is on
// its own BLE characteristic — it already fits in a single notification.
// ─────────────────────────────────────────────────────────────
struct __attribute__((packed)) TelemetryPacket {
    uint32_t seq;   // rolling counter, incremented per notification
    GpsData  gps;   // 18 bytes — latest GPS snapshot
    ImuData  imu;   // 16 bytes — latest accelerometer sample
};                  // 4 + 18 + 16 = 38 bytes

// Must fit in one BLE notification. The nRF52840 negotiates an ATT MTU of up
// to 247 bytes of payload; 38 bytes leaves ample headroom.
static_assert(sizeof(TelemetryPacket) <= 247,
              "TelemetryPacket must fit in one BLE notification (<= 247 B)");
// Guard against accidental padding creeping into the wire layout.
static_assert(sizeof(TelemetryPacket) == 38,
              "TelemetryPacket has unexpected size/padding");
