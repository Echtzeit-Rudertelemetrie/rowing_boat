#include <Arduino.h>
#include "UartReceiver.h"

UartReceiver receiver;

void setup() {
    // USB Serial für den PC (Serial Monitor in PlatformIO)
    Serial.begin(115200);
    
    // Warte kurz, bis USB bereit ist (optional, gut zum Debuggen)
    while(!Serial && millis() < 3000); 

    // Initialisiere den UART Receiver
    receiver.begin();
}

void loop() {
    // Receiver updaten (liest den UART aus)
    receiver.update();

    // Prüfen, ob ein neues Paket vollständig empfangen wurde
    if (receiver.isNewDataAvailable()) {
        MeasurementPack data = receiver.getLatestPacket();

        // Daten auf dem Serial Monitor des PCs ausgeben
        Serial.println("--- Neues Paket empfangen ---");
        Serial.printf("ID / Seq: %u\n", data.espIdAndSeqenceNum);
        
        Serial.print("Force Values: ");
        for (int i = 0; i < PACKET_VALUES; i++) {
            Serial.print(data.force_values[i]);
            Serial.print(" ");
        }
        Serial.println();

        Serial.print("Angle Values: ");
        for (int i = 0; i < PACKET_VALUES; i++) {
            Serial.print(data.angle_values[i]);
            Serial.print(" ");
        }
        Serial.println("\n");

        // Lokale Kopie der empfangenen Daten im Arbeitsspeicher löschen
        memset(&data, 0, sizeof(MeasurementPack));
    }
}