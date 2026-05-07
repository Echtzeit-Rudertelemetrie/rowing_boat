#include <Arduino.h>
#include "espnow.h"

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("ESP-NOW Receiver – getrennte Statistiken fuer Board 1 und 2");
    Serial.printf("Paketgroesse: %u Bytes, %d Werte/Paket, %d-fache Redundanz\n",
                 sizeof(RowingPacket), PACKET_VALUES, PACKET_RETRIES);

    espnow_init_receiver();

    uint8_t local_mac[6];
    WiFi.macAddress(local_mac);
    Serial.printf("Meine MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  local_mac[0], local_mac[1], local_mac[2],
                  local_mac[3], local_mac[4], local_mac[5]);
    Serial.println("Warte auf ESP-NOW Pakete...\n");
}

void loop() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < 5000) return;
    lastPrint = millis();

    espnow_print_stats();
}