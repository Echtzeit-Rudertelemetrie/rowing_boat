#include "ble_telemetry.h"

// ── Backend selection ─────────────────────────────────────────
// ESP32 (planned XIAO ESP32-S3) -> NimBLE; everything else (the current
// XIAO nRF52840) -> Adafruit Bluefruit. Picking "else = Bluefruit" keeps the
// nRF52 build working regardless of which nRF52 macro the core happens to set.
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define BLE_BACKEND_NIMBLE 1
#else
  #define BLE_BACKEND_BLUEFRUIT 1
#endif

// Logical 128-bit UUIDs (human-readable form):
//   service   a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   telemetry a1b2c3d4-0002-4a2b-9c3d-1234567890ab
//   rowing    a1b2c3d4-0003-4a2b-9c3d-1234567890ab
#define BLE_UUID_SERVICE   "a1b2c3d4-0001-4a2b-9c3d-1234567890ab"
#define BLE_UUID_TELEMETRY "a1b2c3d4-0002-4a2b-9c3d-1234567890ab"
#define BLE_UUID_ROWING    "a1b2c3d4-0003-4a2b-9c3d-1234567890ab"
#define BLE_DEVICE_NAME    "RowingBoat"
#define BLE_MAX_CONN       4

// ══════════════════════════════════════════════════════════════
#if defined(BLE_BACKEND_BLUEFRUIT)
// ══════════════════════════════════════════════════════════════
#include <bluefruit.h>

namespace {
    // Bluefruit takes 128-bit UUIDs as little-endian byte arrays (reversed
    // from the human-readable form above; only byte [10] differs: 01/02/03).
    const uint8_t UUID_SVC[16]   = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                    0x2b,0x4a,0x01,0x00,0xd4,0xc3,0xb2,0xa1};
    const uint8_t UUID_TELEM[16] = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                    0x2b,0x4a,0x02,0x00,0xd4,0xc3,0xb2,0xa1};
    const uint8_t UUID_ROW[16]   = {0xab,0x90,0x78,0x56,0x34,0x12,0x3d,0x9c,
                                    0x2b,0x4a,0x03,0x00,0xd4,0xc3,0xb2,0xa1};

    BLEService        svc(UUID_SVC);
    BLECharacteristic chrTelem(UUID_TELEM);
    BLECharacteristic chrRowing(UUID_ROW);

    void onConnect(uint16_t /*conn_handle*/) {
        // Keep advertising while peripheral slots remain so more phones join.
        if (Bluefruit.connected() < BLE_MAX_CONN) Bluefruit.Advertising.start(0);
    }
}

void setupBLE() {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); // raise MTU so 133 B notify fits
    Bluefruit.begin(BLE_MAX_CONN, 0);             // up to 4 peripheral links
    Bluefruit.setName(BLE_DEVICE_NAME);
    Bluefruit.setTxPower(4);
    Bluefruit.Periph.setConnectCallback(onConnect);

    svc.begin();

    chrTelem.setProperties(CHR_PROPS_NOTIFY);
    chrTelem.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chrTelem.setFixedLen(sizeof(TelemetryPacket));
    chrTelem.begin();

    chrRowing.setProperties(CHR_PROPS_NOTIFY);
    chrRowing.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    chrRowing.setFixedLen(sizeof(RowingPacket));
    chrRowing.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(svc);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);   // units of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);               // 0 = advertise forever
}

void bleNotifyTelemetry(const TelemetryPacket& pkt) {
    if (Bluefruit.connected()) chrTelem.notify(&pkt, sizeof(pkt));
}
void bleNotifyRowing(const RowingPacket& pkt) {
    if (Bluefruit.connected()) chrRowing.notify(&pkt, sizeof(pkt));
}
uint8_t bleConnectionCount() { return Bluefruit.connected(); }

// ══════════════════════════════════════════════════════════════
#elif defined(BLE_BACKEND_NIMBLE)
// ══════════════════════════════════════════════════════════════
// FUTURE: Seeed XIAO ESP32-S3. That env's platformio.ini must add:
//   lib_deps    += h2zero/NimBLE-Arduino @ ^1.4.3
//   build_flags += -D CONFIG_BT_NIMBLE_MAX_CONNECTIONS=4
// Untested until the ESP32-S3 hardware is in hand.
#include <NimBLEDevice.h>

namespace {
    NimBLEServer*         server    = nullptr;
    NimBLECharacteristic* chrTelem  = nullptr;
    NimBLECharacteristic* chrRowing = nullptr;
}

void setupBLE() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(247);                 // 133 B notify fits
    server = NimBLEDevice::createServer();
    NimBLEService* svc = server->createService(BLE_UUID_SERVICE);
    chrTelem  = svc->createCharacteristic(BLE_UUID_TELEMETRY, NIMBLE_PROPERTY::NOTIFY);
    chrRowing = svc->createCharacteristic(BLE_UUID_ROWING,    NIMBLE_PROPERTY::NOTIFY);
    svc->start();
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_UUID_SERVICE);
    adv->setScanResponse(true);
    adv->start();
}

void bleNotifyTelemetry(const TelemetryPacket& pkt) {
    if (server && server->getConnectedCount()) {
        chrTelem->setValue((const uint8_t*)&pkt, sizeof(pkt));
        chrTelem->notify();
    }
}
void bleNotifyRowing(const RowingPacket& pkt) {
    if (server && server->getConnectedCount()) {
        chrRowing->setValue((const uint8_t*)&pkt, sizeof(pkt));
        chrRowing->notify();
    }
}
uint8_t bleConnectionCount() {
    return server ? (uint8_t)server->getConnectedCount() : 0;
}

#endif
