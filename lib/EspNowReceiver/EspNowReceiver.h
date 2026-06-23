#pragma once
#include "AppTypes.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class EspNowReceiver {
public:
    EspNowReceiver();
    ~EspNowReceiver();

    // Initialisiert WLAN und ESP-NOW
    bool begin();

    // Holt ein Paket aus der Queue (nicht blockierend)
    // Gibt true zurück, wenn ein Paket empfangen wurde
    bool getPacket(MeasurementPack& outPacket);

private:
    QueueHandle_t packetQueue;
    
    // Statische Methode für den C-Callback von ESP-IDF
#if ESP_IDF_VERSION_MAJOR >= 5
    static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
#else
    static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
#endif

    // Pointer auf die aktuelle Instanz (für den statischen Callback)
    static EspNowReceiver* instance;
};