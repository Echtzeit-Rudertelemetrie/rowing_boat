#include <Arduino.h>
#include <NimBLEDevice.h>

#define DEVICE_NAME "RowingBoat-BLE"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_DATA_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_HELLO_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pDataChar = nullptr;
NimBLECharacteristic* pHelloChar = nullptr;

uint64_t counter = 0;
uint32_t lastNotify = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
        Serial.printf("Client verbunden: %s | aktiv: %d\n",
            NimBLEAddress(desc->peer_ota_addr).toString().c_str(),
            s->getConnectedCount());
        NimBLEDevice::startAdvertising();
        pHelloChar->setValue("hello from esp");
        pHelloChar->notify();
    }
    void onDisconnect(NimBLEServer* s) override {
        Serial.printf("Client weg, noch %d verbunden\n", s->getConnectedCount());
    }
};

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BLE Phase 1 ===");

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(247);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pDataChar = pService->createCharacteristic(
        CHAR_DATA_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pHelloChar = pService->createCharacteristic(
        CHAR_HELLO_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setName(DEVICE_NAME);
    adv->setScanResponse(true);
    adv->start();

    Serial.println("Advertising gestartet, warte auf Handys...");
}

void loop() {
    if (millis() - lastNotify > 1000) {
        lastNotify = millis();
        counter++;

        if (pServer->getConnectedCount() > 0) {
            pDataChar->setValue((uint8_t*)&counter, 8);
            pDataChar->notify();
            Serial.printf("Notify #%llu an %d Clients\n", counter, pServer->getConnectedCount());
        }
    }
}