#include "uart_rx.h"

namespace {
    constexpr uint8_t       SYNC0     = 0xAA;
    constexpr uint8_t       SYNC1     = 0x55;
    constexpr unsigned long UART_BAUD = 115200;

    enum class RxState { Sync0, Sync1, Payload, Crc };
    RxState  state = RxState::Sync0;

    uint8_t  payload[sizeof(RowingPacket)];
    uint16_t payloadIdx = 0;
    uint8_t  crcBytes[2];
    uint8_t  crcIdx = 0;

    RowingPacket latest[UART_RX_MAX_BOARDS];
    bool         haveLatest[UART_RX_MAX_BOARDS] = {false};

    uint32_t goodFrames = 0;
    uint32_t crcErrors  = 0;

    // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0.
    uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t b = 0; b < 8; b++) {
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                     : (uint16_t)(crc << 1);
            }
        }
        return crc;
    }

    void commitFrame() {
        uint16_t rxCrc = (uint16_t)crcBytes[0] | ((uint16_t)crcBytes[1] << 8); // LE
        if (rxCrc != crc16(payload, sizeof(RowingPacket))) {
            crcErrors++;
            return;
        }
        RowingPacket pkt;
        memcpy(&pkt, payload, sizeof(pkt));
        if (pkt.board_id < UART_RX_MAX_BOARDS) {
            latest[pkt.board_id]     = pkt;
            haveLatest[pkt.board_id] = true;
        }
        goodFrames++;
    }
}

void setupUartRx() {
    Serial1.begin(UART_BAUD);
}

void updateUartRx() {
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        switch (state) {
        case RxState::Sync0:
            if (b == SYNC0) state = RxState::Sync1;
            break;
        case RxState::Sync1:
            if (b == SYNC1)      { state = RxState::Payload; payloadIdx = 0; }
            else if (b == SYNC0) { /* stay; could be the real sync start */ }
            else                 { state = RxState::Sync0; }
            break;
        case RxState::Payload:
            payload[payloadIdx++] = b;
            if (payloadIdx >= sizeof(RowingPacket)) { state = RxState::Crc; crcIdx = 0; }
            break;
        case RxState::Crc:
            crcBytes[crcIdx++] = b;
            if (crcIdx >= 2) { commitFrame(); state = RxState::Sync0; }
            break;
        }
    }
}

bool uartRxGetLatest(uint8_t board_id, RowingPacket& out) {
    if (board_id >= UART_RX_MAX_BOARDS || !haveLatest[board_id]) return false;
    out = latest[board_id];
    return true;
}

uint32_t uartRxGoodFrames() { return goodFrames; }
uint32_t uartRxCrcErrors()  { return crcErrors;  }
