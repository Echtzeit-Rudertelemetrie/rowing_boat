#pragma once

#include <Arduino.h>

class Sensor {
public:
  void begin();
  float ReadSensor1();
  float ReadSensor2();

private:
  static constexpr uint8_t SENSOR1_PIN = 34; // Beispiel: ADC1
  static constexpr uint8_t SENSOR2_PIN = 35; // Beispiel: ADC1
};