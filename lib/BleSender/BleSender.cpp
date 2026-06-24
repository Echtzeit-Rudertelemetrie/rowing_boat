#include "BleSender.h"

bool BleSender::begin() {
    Bluefruit.begin(BLE_MAX_CONN, 0);
    Bluefruit.setName("RowingBoat");
    Bluefruit.Periph.setConnectCallback(_connectCb);
    Bluefruit.Periph.setDisconnectCallback(_disconnectCb);

    _svc.begin();
    //_telChr.begin();
    _measChr.begin();

    _advertise();
    return true;
}

// void BleSender::notifyTelemetry(const TelemetryPacket& pkt) {
//     _telChr.notify((const uint8_t*)&pkt, sizeof(pkt));
// }

void BleSender::notifyMeasurement(const MeasurementPack& pkt) {
    _measChr.notify((const uint8_t*)&pkt, sizeof(pkt));
}

uint8_t BleSender::connectionCount() {
    return Bluefruit.connected();
}

void BleSender::_advertise() {
    auto& a = Bluefruit.Advertising;
    a.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    a.addTxPower();
    a.addService(_svc);
    Bluefruit.ScanResponse.addName();
    a.restartOnDisconnect(true);
    a.setInterval(32, 244);
    a.setFastTimeout(30);
    a.start(0);
}

void BleSender::_connectCb(uint16_t conn_handle) {
    Bluefruit.Connection(conn_handle)->requestMtuExchange(BLE_ATT_MTU);
}

void BleSender::_disconnectCb(uint16_t conn_handle, uint8_t reason) {
    (void) conn_handle;
    (void) reason;
}
