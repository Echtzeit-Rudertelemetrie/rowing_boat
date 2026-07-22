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

  if (!forceReader.begin()) {
    Serial.println("ForceReader: AD7124 nicht gefunden - Kraft bleibt 0.");
  }
}

float Sensor::ReadForce() {
  return forceReader.sampleForce();
}

float Sensor::ReadAngle() {
  return angleReader.sampleAndCalculateAngle();
}