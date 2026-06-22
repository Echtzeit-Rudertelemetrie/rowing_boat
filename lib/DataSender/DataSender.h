#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <climits>
#include "AppTypes.h"

constexpr std::uint8_t PACKET_RETRIES = 3;
constexpr std::uint8_t ESP_ID = 1;

constexpr float FORCE_MIN_N   = 0.0f; //TODO: setup sinnvolle Werte hierfür
constexpr float FORCE_MAX_N   = 1000.0f;
constexpr float ANGLE_MIN_DEG = -90.0f;
constexpr float ANGLE_MAX_DEG =  90.0f;

class DataSender {
public:
    explicit DataSender(MeasurementData& data);

    void espnow_init_sender();
    void sendData();

private:
    MeasurementData* data_{nullptr};
};