#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "AppTypes.h"

// NEO-M8T NMEA-Empfaenger am zweiten Hardware-UART des BLE-XIAO ESP32S3.
// GPS TX -> XIAO D10/GPIO9 (RX), GPS RX -> D9/GPIO8 (TX, optional).
// Serial1 auf D7/D6 bleibt dadurch exklusiv fuer den WROOM-UART.
class UartGps {
public:
  void begin();
  void update();
  GpsData data();
  uint32_t charsProcessed() const;
  uint32_t passedChecksums() const;
  uint32_t failedChecksums() const;
  uint32_t locationAgeMs() const;

private:
  static constexpr uint32_t BAUD = 9600;
  static constexpr int RX_PIN = 9;  // XIAO D10, verbunden mit GPS TX
  static constexpr int TX_PIN = 8;  // XIAO D9, verbunden mit GPS RX

  TinyGPSPlus gps_;
};
