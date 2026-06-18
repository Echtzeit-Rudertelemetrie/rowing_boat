#include "EspNowReceiver.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

EspNowReceiver* EspNowReceiver::instance = nullptr;

EspNowReceiver::EspNowReceiver() {
    // Queue für maximal 10 Pakete erstellen
    packetQueue = xQueueCreate(10, sizeof(MeasurementPack));
    instance = this;
}

EspNowReceiver::~EspNowReceiver() {
    vQueueDelete(packetQueue);
    instance = nullptr;
}

bool EspNowReceiver::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);

    // Kanal setzen (Promiscuous mode wird kurz benötigt, wenn man nicht verbunden ist)
    esp_wifi_set_promiscuous(true);
    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println("FEHLER: ESP-NOW Kanal konnte nicht gesetzt werden.");
        esp_wifi_set_promiscuous(false);
        return false;
    }
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("FEHLER: esp_now_init() fehlgeschlagen.");
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW Empfänger erfolgreich initialisiert.");
    return true;
}

bool EspNowReceiver::getPacket(MeasurementPack& outPacket) {
    // Prüfen ob Daten in der Queue sind (Wartezeit 0 = nicht blockierend)
    if (xQueueReceive(packetQueue, &outPacket, 0) == pdTRUE) {
        return true;
    }
    return false;
}

// --- Callback Implementierung ---
#if ESP_IDF_VERSION_MAJOR >= 5
void EspNowReceiver::onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (recv_info == nullptr || incomingData == nullptr || len != sizeof(MeasurementPack)) return;
#else
void EspNowReceiver::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (mac == nullptr || incomingData == nullptr || len != sizeof(MeasurementPack)) return;
#endif

    if (instance && instance->packetQueue) {
        // Daten in die Queue kopieren (wichtig: FromISR, da wir uns im Wi-Fi Task befinden!)
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(instance->packetQueue, incomingData, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}