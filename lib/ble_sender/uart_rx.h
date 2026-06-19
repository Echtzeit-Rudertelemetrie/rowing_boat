#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// UART receive — oarlock data forwarded from the ESP-NOW aggregator.
//
// Wire on the XIAO: hardware Serial1, RX = D7 (pin 7), TX = D6 (pin 6),
// 115200 8N1. Aggregator (ESP32, 3.3 V) TX -> XIAO D7. No level shifter
// needed (both 3.3 V); common GND required.
//
// Frame protocol (PROPOSED — confirm with the aggregator team):
//
//   [0xAA][0x55][ RowingPacket : 133 B ][ CRC16 : 2 B little-endian ]
//
//   - 0xAA 0x55 sync word: lets the receiver re-sync after a dropped byte.
//   - CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the 133 payload
//     bytes only. Frame = 2 + 133 + 2 = 137 B. At 20 packets/s -> ~2.7 kB/s,
//     ~24 % of a 115200 link, so plenty of headroom.
//
// RowingPacket layout MUST match lib/ESPNOW/espnow.h on the aggregator.
// (espnow.h pulls in ESP32-only headers, so the struct is mirrored here;
//  the static_assert below guards against size drift.)
// ─────────────────────────────────────────────────────────────

#define UART_RX_PACKET_VALUES 32
#define UART_RX_MAX_BOARDS    8   // keep latest packet for board_id 0..7

struct __attribute__((packed)) RowingPacket {
    uint8_t  board_id;
    uint32_t seq;
    uint16_t force[UART_RX_PACKET_VALUES];
    uint16_t angle[UART_RX_PACKET_VALUES];
};
static_assert(sizeof(RowingPacket) == 133,
              "RowingPacket must match lib/ESPNOW/espnow.h (133 bytes, packed)");

// Configure Serial1 for receiving frames.
void setupUartRx();

// Drain and parse any bytes waiting on Serial1. Call frequently from loop().
void updateUartRx();

// Copy the most recent valid packet from `board_id` into `out`.
// Returns false if none received yet, or board_id is out of range.
bool uartRxGetLatest(uint8_t board_id, RowingPacket& out);

// Diagnostics: frames accepted (CRC ok) and frames rejected (CRC mismatch).
uint32_t uartRxGoodFrames();
uint32_t uartRxCrcErrors();
