#pragma once
#include <Arduino.h>

enum class EventType : uint8_t {
  ReadSensor1,
  ReadSensor2,
  SendData
};

struct MeasurementData {
  float sensor1 = 0.0f;
  float sensor2 = 0.0f;
  uint32_t frame = 0;
};