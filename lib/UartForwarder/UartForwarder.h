#pragma once
#include "AppTypes.h"

class UartForwarder {
public:
    // Initialisiert Serial2
    void begin();

    // Sendet das Paket binär an den Empfänger (inkl. Magic Bytes zur Synchronisierung)
    void forwardPacket(const MeasurementPack& pack);

    // Optional: Überprüft, ob vom XIAO Antworten kommen (wie im ursprünglichen Test-Skript)
    void checkIncoming();

private:
    // Magic Bytes für sauberes UART-Framing
    static constexpr uint8_t HEADER_1 = 0xAA;
    static constexpr uint8_t HEADER_2 = 0xBB;
};