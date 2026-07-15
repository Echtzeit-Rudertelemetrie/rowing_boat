#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <climits>
#include "AppTypes.h"

constexpr std::uint8_t PACKET_RETRIES = 3;
constexpr std::uint8_t ESPNOW_CHANNEL = 11;
constexpr std::uint8_t ESP_ID = 1;

constexpr float FORCE_MIN_N   = 0.0f;
constexpr float FORCE_MAX_N   = 1000.0f;
// constexpr float ANGLE_MIN_DEG = -180.0f;
// constexpr float ANGLE_MAX_DEG =  180.0f;

// Kehrwerte zur Compile-Zeit: Division hat auf der LX7-FPU keinen eigenen
// Befehl (mehrzyklige Reziprok-Sequenz) — quantize() multipliziert damit
// stattdessen, 2x pro 100-Hz-Tick.
constexpr float FORCE_INV_SPAN = 1.0f / (FORCE_MAX_N - FORCE_MIN_N);
// constexpr float ANGLE_INV_SPAN = 1.0f / (ANGLE_MAX_DEG - ANGLE_MIN_DEG);

class DataSender {
public:
    explicit DataSender(MeasurementData& data);

    void espnow_init_sender();
    void sendData();

private:
    MeasurementData* data_{nullptr};
};