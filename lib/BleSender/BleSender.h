#pragma once
#include <Arduino.h>
#include <bluefruit.h>
#include "AppTypes.h" // MeasurementPack
#include "Gps.h"      // GpsData
#include "Imu.h"      // ImuData

// BLE broadcast via Adafruit Bluefruit (nRF52840, XIAO Sense).
// One GATT service, ONE NOTIFY characteristic carrying the shared 84-byte
// MeasurementPack. The top 4 bits of idAndSeq tell the client what's inside:
//   id 0      — telemetry (GPS + IMU), ~3.5 Hz
//   id 1..15  — oarlock data forwarded from UART, ~20 Hz
// 128-bit UUIDs:
//   service  a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   packet   a1b2c3d4-0002-4a2b-9c3d-1234567890ab

// struct __attribute__((packed)) TelemetryPacket
// {
//     //uint8_t id; // 4bit id
//     uint32_t id_seq; //  28 bit rolling counter per notification
//     GpsData gps;  // 16 B
//     ImuData imu;  // 16 B
// };

// static_assert(sizeof(TelemetryPacket) == 39, "TelemetryPacket must stay 38 bytes (wire format)");

// MTU budget: notification payload must fit in one ATT packet (MTU - 3).
// MTU 104 × BLE_MAX_CONN=4 fits SoftDevice RAM; MTU 247 × 4 returns NO_MEM
// from sd_ble_enable(). Increase MTU or decrease BLE_MAX_CONN together.
// Override via build_flags: -DBLE_GATT_ATT_MTU_MAX=104
static constexpr uint8_t BLE_MAX_CONN = 4;
static constexpr uint16_t BLE_ATT_MTU = 104;
// static_assert(sizeof(TelemetryPacket) <= BLE_ATT_MTU - 3,
//               "TelemetryPacket must fit one notification");
static_assert(sizeof(MeasurementPack) <= BLE_ATT_MTU - 3,
              "MeasurementPack must fit one notification");

class BleSender
{
public:
    bool begin();
    // void notifyTelemetry(const TelemetryPacket &pkt);
    void notifyMeasurement(const MeasurementPack &pkt);
    uint8_t connectionCount();


private:
    BLEService _svc{"a1b2c3d4-0001-4a2b-9c3d-1234567890ab"};
    // BLECharacteristic _telChr{"a1b2c3d4-0002-4a2b-9c3d-1234567890ab",
    //                           BLERead | BLENotify, sizeof(TelemetryPacket)};
    BLECharacteristic _measChr{"a1b2c3d4-0002-4a2b-9c3d-1234567890ab",
                               BLERead | BLENotify, sizeof(MeasurementPack)};

    void _advertise();

    static void _connectCb(uint16_t conn_handle);
    static void _disconnectCb(uint16_t conn_handle, uint8_t reason);
};
