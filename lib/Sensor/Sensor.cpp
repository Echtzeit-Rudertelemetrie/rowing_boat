#include "Sensor.h"
#include "AngleReader.h"
#include "ForceReader.h"
#include <Wire.h>

void Sensor::begin() {
  // I2C starten, bevor die Sensoren initialisiert werden.
  Wire.begin();
  // 400 kHz Fast-Mode: der ICM-20948 kann das, und beim 5-ms-Event-Takt wird
  // das Auslesen von Accel+Gyro+Mag sonst mit 100 kHz zu knapp.
  Wire.setClock(400000);

  if (!angleReader.begin()) {
    Serial.println("AngleReader: Sensor nicht gefunden - Winkel bleibt auf letztem Wert (0).");
  }
}

float Sensor::ReadForce() {
  // DMS/Kraftsensor ist aktuell nicht angeschlossen -> Dummy-Wert.
  //const int force = forceReader.sampleForce();
  return (1.03234f * 2.3f) / 4095.0f;
}

float Sensor::ReadAngle() {
  return angleReader.sampleAndCalculateAngle();
}