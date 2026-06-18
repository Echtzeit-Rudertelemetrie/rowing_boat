#include "UartForwarder.h"

void UartForwarder::begin() {
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    Serial.printf("UART2 initialisiert auf RX:%d TX:%d Baud:%u\n", UART2_RX_PIN, UART2_TX_PIN, UART_BAUD);
}

void UartForwarder::forwardPacket(const MeasurementPack& pack) {
    // 1. Header senden (hilft dem XIAO, den Start eines Pakets zu erkennen)
    Serial2.write(HEADER_1);
    Serial2.write(HEADER_2);

    // 2. Binäres Struct senden
    Serial2.write(reinterpret_cast<const uint8_t*>(&pack), sizeof(MeasurementPack));
    
    // 3. Flushen, um sicherzugehen, dass alles gesendet wurde
    Serial2.flush();
}

void UartForwarder::checkIncoming() {
    // Lies Zeug vom XIAO, falls dieser Statusmeldungen zurücksendet
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.print(c); // Auf USB-Konsole ausgeben
    }
}