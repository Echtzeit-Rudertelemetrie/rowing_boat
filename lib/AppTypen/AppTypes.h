#pragma once
#include <Arduino.h>

// Konfiguration ESP-NOW
static constexpr uint8_t ESPNOW_CHANNEL = 11;
static constexpr uint8_t PACKET_VALUES = 20;

// Konfiguration UART
static constexpr int UART2_RX_PIN = 16;
static constexpr int UART2_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;

// Das exakt gleiche Struct wie beim Sender definieren
typedef struct __attribute__((packed)) {
    uint32_t espIdAndSequenceNum;
    uint16_t force_values[PACKET_VALUES];
    uint16_t angle_values[PACKET_VALUES];
} MeasurementPack;