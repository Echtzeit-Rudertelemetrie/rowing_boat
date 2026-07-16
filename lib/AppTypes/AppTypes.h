#pragma once
#include <Arduino.h>

#define ESPNOW_MAX_BOARD_ID 255

// Konfiguration ESP-NOW
static constexpr uint8_t ESPNOW_CHANNEL = 11;
// 32 Werte pro Paket -> MeasurementPack = 4 + 32*2 + 32*2 = 132 Bytes.
// Muss mit dem Sender (Branch sensor_miniesp_code / DataSender) uebereinstimmen,
// sonst verwirft der Empfaenger wegen Laengenpruefung (len != sizeof) alles.
// Achtung: bei Aenderung auch BLE_ATT_MTU in lib/BleSender/BleSender.h anpassen
// (MeasurementPack muss in eine BLE-Notification passen).
static constexpr uint8_t PACKET_VALUES = 32;

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
  u_int16_t force_values[PACKET_VALUES];
  u_int16_t angle_values[PACKET_VALUES];
} MeasurementPack;

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
};