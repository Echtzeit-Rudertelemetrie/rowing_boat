#include <Arduino.h>
#include <NimBLEDevice.h>

#define DEVICE_NAME "RowingBoat-BLE"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_DATA_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_HELLO_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// --- Streaming Config ---
static const uint16_t SAMPLES_PER_SECOND = 800;
static const uint16_t SAMPLES_PER_PACKET = 80; // 20 Pakete/s
static const uint16_t PACKET_INTERVAL_MS = 1000 * SAMPLES_PER_PACKET / SAMPLES_PER_SECOND; // 50ms

struct __attribute__((packed)) DataPacket {
    uint32_t seq; // Sequenznummer
    uint32_t samples[SAMPLES_PER_PACKET]; // 20 x 32bit Zufallsdaten
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
        // Weiter advertisern für die nächsten 3 Handys
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
    Serial.println("\n=== BLE Phase 2 - 400x32bit/s ===");

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max Power
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

    Serial.printf("Advertising gestartet. Paket: %d Bytes alle %d ms\n",
                  sizeof(DataPacket), PACKET_INTERVAL_MS);
}

void loop() {
    // Nichts tun wenn keiner verbunden
    if (pServer->getConnectedCount() == 0) {
        delay(100);
        return;
    }

    uint32_t now = millis();
    if (now - lastPacketTime >= PACKET_INTERVAL_MS) {
        lastPacketTime = now;

        DataPacket pkt;
        pkt.seq = packetSeq++;
        for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
            pkt.samples[i] = esp_random(); // echte Zufallsdaten, 32bit
        }

        pDataChar->setValue((uint8_t*)&pkt, sizeof(pkt));
        pDataChar->notify();
        packetsSent++;
    }

    // Statistik jede Sekunde
    if (now - lastStatsTime >= 1000) {
        lastStatsTime = now;
        Serial.printf("Sent %u Pakete/s, seq=%u, Clients=%d\n",
                      packetsSent, packetSeq-1, pServer->getConnectedCount());
        packetsSent = 0;
    }
}