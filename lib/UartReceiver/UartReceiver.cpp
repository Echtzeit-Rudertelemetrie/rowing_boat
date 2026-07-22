#include "UartReceiver.h"

void UartReceiver::begin() {
    // XIAO ESP32S3: Serial1 muss explizit auf die D6/D7-Pins gelegt werden
    // (anders als beim nRF52840, wo D6/D7 fest mit Serial1 verdrahtet sind).
    //   D7 = GPIO44 = RX  <- ESP32 GPIO17 (TX2)
    //   D6 = GPIO43 = TX  -> ESP32 GPIO16 (RX2)
    static constexpr int XIAO_UART_RX_PIN = 44;  // D7
    static constexpr int XIAO_UART_TX_PIN = 43;  // D6
    Serial1.begin(UART_BAUD, SERIAL_8N1, XIAO_UART_RX_PIN, XIAO_UART_TX_PIN);
    Serial.println("XIAO ESP32S3 UART-Receiver initialisiert (Serial1 RX=44/D7 TX=43/D6, Baud: 115200).");
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
    
    // Temporäre Kopie für die Rückgabe erstellen
    MeasurementPack tempPacket = latestPacket;
    
    // Gespeicherte Daten im Receiver-Objekt löschen
    memset(&latestPacket, 0, sizeof(MeasurementPack));
    memset(buffer, 0, sizeof(buffer));
    bufferIndex = 0;
    
    return tempPacket;
}