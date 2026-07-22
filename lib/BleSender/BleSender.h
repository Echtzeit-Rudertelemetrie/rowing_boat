#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "AppTypes.h" // MeasurementPack

// BLE broadcast via NimBLE-Arduino (ESP32-S3, Seeed XIAO ESP32S3).
// One GATT service, ONE NOTIFY characteristic carrying the shared 84-byte
// MeasurementPack. The top 3 bits of espIdAndSeqenceNum tell the client what's
// inside (matches DataSender's ESP_ID<<29 layout):
//   id 0      - telemetry (GPS), assembled locally on the hub
//   id 1..7   - oarlock data forwarded from UART
// 128-bit UUIDs:
//   service  a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   packet   a1b2c3d4-0002-4a2b-9c3d-1234567890ab

static constexpr uint8_t  BLE_MAX_CONN = 4;
// Must fit MeasurementPack (132 B at PACKET_VALUES=32) + 3B ATT notify header.
// ESP32/NimBLE handles large MTU fine; the phone negotiates min(this, its own).
// iOS defaults to 185, modern Android up to 517, so 185 is safely negotiable.
static constexpr uint16_t BLE_ATT_MTU  = 185;

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
