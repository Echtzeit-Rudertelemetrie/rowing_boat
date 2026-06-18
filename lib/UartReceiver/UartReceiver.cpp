#include "UartReceiver.h"

void UartReceiver::begin() {
    // Beim XIAO sind die Pins D6 (TX) und D7 (RX) hardwaremäßig mit Serial1 verknüpft
    Serial1.begin(UART_BAUD);
    Serial.println("XIAO UART-Receiver initialisiert (Baud: 115200).");
}

void UartReceiver::update() {
    // Solange Daten im UART-Puffer liegen, Byte für Byte abarbeiten
    while (Serial1.available() > 0) {
        uint8_t incomingByte = Serial1.read();

        switch (state) {
            case RxState::WAIT_HEADER_1:
                if (incomingByte == HEADER_1) {
                    state = RxState::WAIT_HEADER_2;
                }
                break;

            case RxState::WAIT_HEADER_2:
                if (incomingByte == HEADER_2) {
                    // Beide Header gefunden, starte mit dem Einlesen des Structs
                    state = RxState::READ_PAYLOAD;
                    bufferIndex = 0;
                } else if (incomingByte == HEADER_1) {
                    // Falls die Sequenz AA AA BB war
                    state = RxState::WAIT_HEADER_2;
                } else {
                    // Falsches Byte, zurück zum Anfang
                    state = RxState::WAIT_HEADER_1;
                }
                break;

            case RxState::READ_PAYLOAD:
                // Schreibe das Byte in den Puffer
                buffer[bufferIndex++] = incomingByte;

                // Prüfe, ob das gesamte Struct empfangen wurde
                if (bufferIndex >= sizeof(MeasurementPack)) {
                    // Kopiere den Puffer in das fertige Struct
                    memcpy(&latestPacket, buffer, sizeof(MeasurementPack));
                    
                    newData = true; // Flag setzen, dass neue Daten da sind
                    state = RxState::WAIT_HEADER_1; // Zurücksetzen für das nächste Paket
                    
                    // Optional: Rückmeldung an den ESP32 senden (checkIncoming liest das aus)
                    Serial1.println("OK"); 
                }
                break;
        }
    }
}

bool UartReceiver::isNewDataAvailable() {
    return newData;
}

MeasurementPack UartReceiver::getLatestPacket() {
    newData = false; // Flag zurücksetzen, da Daten nun abgeholt werden
    return latestPacket;
}