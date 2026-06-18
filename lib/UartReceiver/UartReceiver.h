#pragma once
#include <Arduino.h>
#include "AppTypes.h"

class UartReceiver {
public:
    // Initialisiert den UART (Serial1 beim XIAO)
    void begin();

    // Muss regelmäßig im loop() aufgerufen werden
    void update();

    // Gibt true zurück, wenn ein neues, komplettes Paket empfangen wurde
    bool isNewDataAvailable();

    // Holt das zuletzt empfangene Paket
    MeasurementPack getLatestPacket();

private:
    static constexpr uint8_t HEADER_1 = 0xAA;
    static constexpr uint8_t HEADER_2 = 0xBB;
    static constexpr uint32_t UART_BAUD = 115200;

    // Zustände für die State Machine
    enum class RxState {
        WAIT_HEADER_1,
        WAIT_HEADER_2,
        READ_PAYLOAD
    };

    RxState state = RxState::WAIT_HEADER_1;
    
    uint8_t buffer[sizeof(MeasurementPack)];
    size_t bufferIndex = 0;
    
    MeasurementPack latestPacket;
    bool newData = false;
};