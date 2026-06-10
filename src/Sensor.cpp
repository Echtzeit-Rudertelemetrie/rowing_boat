#include "Sensor.h"

void Sensor::begin() {
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
}

float Sensor::ReadSensor1() {
  const int raw = analogRead(SENSOR1_PIN);
  return (raw * 3.3f) / 4095.0f;
}

float Sensor::ReadSensor2() {
  const int raw = analogRead(SENSOR2_PIN);
  return (raw * 3.3f) / 4095.0f;
}