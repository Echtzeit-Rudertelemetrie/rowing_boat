#pragma once
#include <Arduino.h>
#include "AppTypes.h"   // MeasurementPack (oarlock data forwarded from UART)
#include "Gps.h"        // GpsData
#include "Imu.h"        // ImuData
#include "Battery.h"    // BatteryData

// BLE broadcast via Adafruit Bluefruit (nRF52840, XIAO Sense).
// One GATT service, two NOTIFY characteristics:
//   TelemetryPacket (38 B) — GPS + IMU, ~3.5 Hz
//   MeasurementPack (84 B) — oarlock data forwarded from UART, ~20 Hz
// 128-bit UUIDs:
//   service     a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   telemetry   a1b2c3d4-0002-4a2b-9c3d-1234567890ab
//   measurement a1b2c3d4-0003-4a2b-9c3d-1234567890ab

// GPS + IMU snapshot pushed to phones (38 bytes, packed). Battery dropped — the
// pack only charges, it never reports state.
struct __attribute__((packed)) TelemetryPacket {
    uint32_t    seq;       // rolling counter per notification
    GpsData     gps;       // 18 B
    ImuData     imu;       // 16 B
};
static_assert(sizeof(TelemetryPacket) == 38, "TelemetryPacket must stay 38 bytes (wire format)");

// Negotiated max ATT MTU. Kept modest (not 247) so the per-connection SoftDevice
// RAM fits BLE_MAX_CONN connections — MTU 247 × 4 connections returns NO_MEM from
// sd_ble_enable(). A notification of N bytes needs MTU >= N + 3 (ATT header), so
// both packets must fit BLE_ATT_MTU - 3.
static constexpr uint16_t BLE_ATT_MTU = 104;
static_assert(sizeof(TelemetryPacket) <= BLE_ATT_MTU - 3, "TelemetryPacket must fit one notification");
static_assert(sizeof(MeasurementPack) <= BLE_ATT_MTU - 3, "MeasurementPack must fit one notification");

class BleSender {
public:
    bool    begin();                                       // false if the SoftDevice failed to come up
    void    notifyTelemetry(const TelemetryPacket& pkt);   // GPS+IMU
    void    notifyMeasurement(const MeasurementPack& pkt); // forwarded oarlock data
    uint8_t connectionCount();
};
