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

    // Der Sender schickt jedes logische Paket PACKET_RETRIES-mal (identische
    // seq). Hier deduplizieren wir per Board-ID, damit UART/BLE jedes Paket nur
    // einmal bekommen. Groesse an die ID-Breite aus AppTypes.h gekoppelt: die
    // ID indiziert diese Arrays direkt, ein zu kleines Array waere ein
    // Schreibzugriff hinter das Ende — und das im ISR-Kontext.
    static constexpr uint8_t MAX_BOARD_IDS = IDSEQ_ID_MASK + 1;
    volatile uint32_t lastSeqPerId[MAX_BOARD_IDS];
    volatile bool     seenPerId[MAX_BOARD_IDS];

    // Statische Methode für den C-Callback von ESP-IDF
#if ESP_IDF_VERSION_MAJOR >= 5
    static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
#else
    static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
#endif

    // Pointer auf die aktuelle Instanz (für den statischen Callback)
    static EspNowReceiver* instance;
};