#include <Arduino.h>
#include "AppTypes.h"
#include "EspNowReceiver.h"
#include "UartForwarder.h"

// Instanzen unserer Klassen
EspNowReceiver espReceiver;
UartForwarder uartBridge;

void setup() {
    // USB-Konsole für Debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32 ESP-NOW -> UART Bridge ===");

    // UART-Verbindung zum XIAO starten
    uartBridge.begin();

    // ESP-NOW Empfänger starten
    if (!espReceiver.begin()) {
        Serial.println("Kritischer Fehler bei ESP-NOW Initialisierung. Neustart...");
        delay(3000);
        ESP.restart();
    }
}

void loop() {
    MeasurementPack incomingPack;

    // 1. Prüfen, ob ein neues Paket via ESP-NOW reingekommen ist
    if (espReceiver.getPacket(incomingPack)) {
        
        // --- Debug Ausgabe auf USB ---
        const uint8_t espId = static_cast<uint8_t>((incomingPack.espIdAndSeqenceNum >> 29) & 0x07u);
        const uint32_t seq  = incomingPack.espIdAndSeqenceNum & 0x1FFFFFFFu;
        Serial.printf("[RX] ESP-ID: %u | Seq: %lu -> Leite via UART weiter...\n", espId, static_cast<unsigned long>(seq));

        // 2. Paket unverändert via UART2 an den XIAO weiterleiten
        uartBridge.forwardPacket(incomingPack);
    }
    else {
        delay(45);
        for(int i = 0; i < PACKET_VALUES; i++)
        {
            incomingPack.angle_values[i] = (uint16_t) i;
            incomingPack.force_values[i] = (uint16_t) i;
        }
        incomingPack.espIdAndSeqenceNum = (uint32_t) 764874;

        uartBridge.forwardPacket(incomingPack);
    }

    // 3. (Optional) Schauen, ob der XIAO etwas zurückgesendet hat (Diagnose)
    uartBridge.checkIncoming();

    // Kurzes Delay um den Watchdog nicht zu triggern
    delay(1); 
}