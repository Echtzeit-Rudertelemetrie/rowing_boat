#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "AppTypes.h" // MeasurementPack

// BLE broadcast via NimBLE-Arduino (ESP32-S3, Seeed XIAO ESP32S3).
// One GATT service, ONE NOTIFY characteristic carrying the shared 36-byte
// MeasurementPack. The top 4 bits of espIdAndSeqenceNum tell the client what's
// inside (matches the shared packIdSeq helper):
//   id 0      - telemetry (GPS), assembled locally on the hub
//   id 1..15  - oarlock data forwarded from UART
// 128-bit UUIDs:
//   service  a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   packet   a1b2c3d4-0002-4a2b-9c3d-1234567890ab

static constexpr uint8_t  BLE_MAX_CONN = 4;
// Must fit MeasurementPack (36 B at PACKET_VALUES=8) + 3B ATT notify header.
static constexpr uint16_t BLE_ATT_MTU  = 64;

static_assert(sizeof(MeasurementPack) <= BLE_ATT_MTU - 3,
              "MeasurementPack must fit one notification (raise BLE_ATT_MTU or lower PACKET_VALUES)");

class BleSender
{
public:
    // Bring up the NimBLE stack, GATT server and advertising. Returns false if
    // the characteristic could not be created or advertising failed to start.
    bool    begin();

    // Send the packet to every subscribed client as a NOTIFY.
    void    notifyMeasurement(const MeasurementPack& pkt);

    // Number of currently connected central devices (phones).
    uint8_t connectionCount();

private:
    static constexpr const char* SERVICE_UUID = "a1b2c3d4-0001-4a2b-9c3d-1234567890ab";
    static constexpr const char* PACKET_UUID  = "a1b2c3d4-0002-4a2b-9c3d-1234567890ab";
    static constexpr const char* DEVICE_NAME  = "RowingBoat";

    NimBLEServer*         _server  = nullptr;
    NimBLECharacteristic* _measChr = nullptr;
};
