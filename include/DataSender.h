#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include "AppTypes.h"

#define PACKET_RETRIES 3
#define ESPNOW_CHANNEL 11

constexpr uint8_t ESP_ID = 1;

constexpr float FORCE_MIN_N     =    0.0f;
constexpr float FORCE_MAX_N     = 1000.0f;
constexpr float ANGLE_MIN_DEG   =  -90.0f;
constexpr float ANGLE_MAX_DEG   =  +90.0f;

class DataSender {
public:
  explicit DataSender(MeasurementData& data);

  void sendData();

private:
  MeasurementData* data_;
};