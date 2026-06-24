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

// One 84-byte packet shared by every source. The top 4 bits of idAndSeq select
// what the two 16-bit regions mean; the lower 28 bits are a rolling sequence:
//   id == 0      -> telemetry:    force_values region holds GpsData,
//                                  angle_values region holds ImuData
//   id 1..15     -> oarlock #id:  force_values / angle_values as named
typedef struct __attribute__((packed)) {
  uint32_t idAndSeq;                      // id:4 (top) | seq:28
  uint16_t force_values[PACKET_VALUES];   // measurement: force | telemetry: GPS bytes
  uint16_t angle_values[PACKET_VALUES];   // measurement: angle | telemetry: IMU bytes
} MeasurementPack;

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
  float imu_onBoard;
  float gps;
};