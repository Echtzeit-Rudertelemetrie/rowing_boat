#include <Arduino.h>
#include <NimBLEDevice.h>

#define DEVICE_NAME "RowingBoat-BLE"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_DATA_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_HELLO_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// === HIER EINSTELLEN ===
static constexpr uint16_t PACKETS_PER_SECOND = 20; // z.B. 10, 20, 50
static constexpr uint16_t VALUES_PER_PACKET = 20; // Anzahl 32-bit Werte pro Paket

// === AUTOMATISCH BERECHNET ===
static constexpr uint16_t PACKET_INTERVAL_MS = 1000 / PACKETS_PER_SECOND;
static constexpr uint32_t SAMPLES_PER_SECOND = (uint32_t)PACKETS_PER_SECOND * VALUES_PER_PACKET;

struct __attribute__((packed)) DataPacket {
    uint32_t seq; // Paketnummer
    uint32_t timestamp_ms; // millis() beim Senden
    uint32_t samples[VALUES_PER_PACKET]; // deine Daten
};

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pDataChar = nullptr;
NimBLECharacteristic* pHelloChar = nullptr;

uint32_t packetSeq = 0;
uint32_t lastPacketTime = 0;
uint32_t packetsSent = 0;
uint32_t lastStatsTime = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
        Serial.printf("Client verbunden: %s | aktiv: %d\n",
            NimBLEAddress(desc->peer_ota_addr).toString().c_str(),
            s->getConnectedCount());
        NimBLEDevice::startAdvertising();
        pHelloChar->setValue("hello from esp phase2");
        pHelloChar->notify();
    }
    void onDisconnect(NimBLEServer* s) override {
        Serial.printf("Client weg, noch %d verbunden\n", s->getConnectedCount());
    }
};

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BLE Phase 2 - mit Timestamp ===");

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

    Serial.printf("Config: %u Pakete/s x %u Werte = %lu Samples/s\n",
                  PACKETS_PER_SECOND, VALUES_PER_PACKET, SAMPLES_PER_SECOND);
    Serial.printf("Paketgroesse: %u Bytes alle %u ms\n",
                  sizeof(DataPacket), PACKET_INTERVAL_MS);

    if (sizeof(DataPacket) > 240) {
        Serial.println("WARNUNG: Paket > 240 Byte! Das ist groesser als die BLE MTU (247).");
        Serial.println(" -> Reduziere VALUES_PER_PACKET auf max 58, sonst wird abgeschnitten.");
    }
}

void loop() {
    if (pServer->getConnectedCount() == 0) {
        delay(100);
        return;
    }

    uint32_t now = millis();
    if (now - lastPacketTime >= PACKET_INTERVAL_MS) {
        lastPacketTime = now;

        DataPacket pkt;
        pkt.seq = packetSeq++;
        pkt.timestamp_ms = now;
        for (int i = 0; i < VALUES_PER_PACKET; i++) {
            pkt.samples[i] = esp_random();
        }

        pDataChar->setValue((uint8_t*)&pkt, sizeof(pkt));
        pDataChar->notify();
        packetsSent++;
    }

    if (now - lastStatsTime >= 1000) {
        lastStatsTime = now;
        Serial.printf("Sent %u Pakete/s, seq=%u, ts=%lu, Clients=%d\n",
                      packetsSent, packetSeq-1, now, pServer->getConnectedCount());
        packetsSent = 0;
    }
}