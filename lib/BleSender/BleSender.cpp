#include "BleSender.h"

// ---------------------------------------------------------------------------
// Server callbacks: keep advertising alive so multiple phones (up to
// BLE_MAX_CONN) can connect, and resume advertising after a disconnect.
// ---------------------------------------------------------------------------
namespace {
class HubServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        (void) connInfo;
        // NimBLE stops advertising once a central connects; restart it so the
        // next phone can still find us, until we hit the connection limit.
        if (pServer->getConnectedCount() < BLE_MAX_CONN) {
            NimBLEDevice::startAdvertising();
        }
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        (void) pServer; (void) connInfo; (void) reason;
        NimBLEDevice::startAdvertising();
    }
};
HubServerCallbacks g_serverCallbacks;
} // namespace

bool BleSender::begin() {
    NimBLEDevice::init(DEVICE_NAME);
    // Request an MTU that fits the 36-byte MeasurementPack in one notification.
    NimBLEDevice::setMTU(BLE_ATT_MTU);

    _server = NimBLEDevice::createServer();
    _server->setCallbacks(&g_serverCallbacks, false);

    NimBLEService* svc = _server->createService(SERVICE_UUID);
    _measChr = svc->createCharacteristic(
        PACKET_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
        sizeof(MeasurementPack));
    svc->start();

    // Advertisement (31 B budget): flags + short name in the primary packet,
    // the 128-bit service UUID (18 B) in the scan response. Splitting avoids an
    // over-31-byte overflow that would make start() fail.
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setName(DEVICE_NAME);
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanData;
    scanData.addServiceUUID(SERVICE_UUID);
    adv->setScanResponseData(scanData);
    adv->enableScanResponse(true);

    bool advOk = adv->start();
    return advOk && (_measChr != nullptr);
}

void BleSender::notifyMeasurement(const MeasurementPack& pkt) {
    if (!_measChr) return;
    _measChr->setValue(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    _measChr->notify(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

uint8_t BleSender::connectionCount() {
    return _server ? _server->getConnectedCount() : 0;
}
