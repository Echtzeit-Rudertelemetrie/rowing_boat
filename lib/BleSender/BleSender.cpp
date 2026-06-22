#include "BleSender.h"
#include <bluefruit.h>

#define BLE_DEVICE_NAME  "RowingBoat"
#define BLE_MAX_CONN     4

// 128-bit UUIDs as little-endian byte arrays (reverse of the human-readable form).
// Byte [10] is the only difference between service / telemetry / measurement.
static const uint8_t UUID_SVC[16]   = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                        0x2b,0x4a,0x01,0x00,0xd4,0xc3,0xb2,0xa1};
static const uint8_t UUID_TELEM[16] = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                        0x2b,0x4a,0x02,0x00,0xd4,0xc3,0xb2,0xa1};
static const uint8_t UUID_MEAS[16]  = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                        0x2b,0x4a,0x03,0x00,0xd4,0xc3,0xb2,0xa1};

static BLEService        svc(UUID_SVC);
static BLECharacteristic chrTelem(UUID_TELEM);
static BLECharacteristic chrMeas(UUID_MEAS);

// Keep advertising after each connection until all BLE_MAX_CONN slots are full,
// so additional phones can still find and join the hub. The guard MUST match
// the slot count passed to Bluefruit.begin() — restarting advertising with no
// free slot destabilises live connections (that was the earlier −127 dBm bug).
static void onConnect(uint16_t /*conn_handle*/) {
    if (Bluefruit.connected() < BLE_MAX_CONN) Bluefruit.Advertising.start(0);
}

bool BleSender::begin() {
    // configPrphConn raises the MTU just enough for the 84-byte MeasurementPack
    // (BLE_ATT_MTU, see header) with the default event length and a small queue.
    // Both knobs scale per-connection RAM, and the SoftDevice's total must fit the
    // linker reserve for BLE_MAX_CONN connections.
    //
    // DO NOT raise these toward configPrphBandwidth(BANDWIDTH_MAX) — that uses
    // MTU 247 + event_len 100, and × 4 connections sd_ble_enable() returns NO_MEM,
    // so Bluefruit.begin() returns false and the hub never advertises. We now
    // CHECK that return and report failure instead of silently continuing.
    Bluefruit.configPrphConn(BLE_ATT_MTU, BLE_GAP_EVENT_LENGTH_DEFAULT, 1, 1);
    if (!Bluefruit.begin(BLE_MAX_CONN, 0)) return false;   // up to 4 phones; false = NO_MEM
    Bluefruit.setName(BLE_DEVICE_NAME);
    Bluefruit.setTxPower(4);
    Bluefruit.Periph.setConnectCallback(onConnect);

    svc.begin();

    chrTelem.setProperties(CHR_PROPS_NOTIFY);
    chrTelem.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chrTelem.setFixedLen(sizeof(TelemetryPacket));
    chrTelem.begin();

    chrMeas.setProperties(CHR_PROPS_NOTIFY);
    chrMeas.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chrMeas.setFixedLen(sizeof(MeasurementPack));
    chrMeas.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(svc);  // 128-bit UUID fills the primary packet
    Bluefruit.ScanResponse.addName();       // name goes in scan response
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
    return true;
}

void BleSender::notifyTelemetry(const TelemetryPacket& pkt) {
    if (Bluefruit.connected()) chrTelem.notify(&pkt, sizeof(pkt));
}

void BleSender::notifyMeasurement(const MeasurementPack& pkt) {
    if (Bluefruit.connected()) chrMeas.notify(&pkt, sizeof(pkt));
}

uint8_t BleSender::connectionCount() { return Bluefruit.connected(); }
