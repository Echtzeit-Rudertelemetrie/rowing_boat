#pragma once
#include <Arduino.h>

#define PACKET_VALUES  32

#define ESPNOW_MAX_BOARD_ID 255

enum class EventType : uint8_t {
  ReadSensor1,
  ReadSensor2,
  SendData
};

typedef struct __attribute__((packed)) {
  u_int16_t force_values[PACKET_VALUES];
  u_int16_t angle_values[PACKET_VALUES];
  uint32_t espIdAndSeqenceNum;
} MeasurementPack;

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
};