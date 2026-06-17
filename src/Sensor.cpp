#include "Sensor.h"
#include "AngleReader.h"
#include "ForceReader.h"
#include <Wire.h>

void Sensor::begin() {
  // I2C starten, bevor die Sensoren initialisiert werden.
  Wire.begin();

  if (!angleReader.begin()) {
    Serial.println("AngleReader: Sensor(en) nicht gefunden - laeuft mit Dummy-Werten weiter.");
  }
}

float Sensor::ReadForce() {
  //const int force = forceReader.sampleForce();
  return (1.03234f * 2.3f) / 4095.0f;
}

float Sensor::ReadAngle() {
  //const float angle = angleReader.sampleAndCalculateAngle();
  return (1.03234f * 3.3f) / 4095.0f;
}