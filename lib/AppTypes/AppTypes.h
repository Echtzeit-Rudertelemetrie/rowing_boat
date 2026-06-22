#pragma once
#include <Arduino.h>

#define ESPNOW_MAX_BOARD_ID 255

// Konfiguration ESP-NOW
static constexpr uint8_t ESPNOW_CHANNEL = 11;
static constexpr uint8_t PACKET_VALUES = 20;

// Konfiguration UART
static constexpr int UART2_RX_PIN = 16;
static constexpr int UART2_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;

enum class EventType : uint8_t {
  ReadSensor1,
  ReadSensor2,
  SendData
};

typedef struct __attribute__((packed)) {
  uint32_t espIdAndSeqenceNum;
  uint16_t force_values[PACKET_VALUES];   // uint16_t (war u_int16_t: BSD-Typ, auf nRF52 nicht definiert)
  uint16_t angle_values[PACKET_VALUES];
} MeasurementPack;

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
};