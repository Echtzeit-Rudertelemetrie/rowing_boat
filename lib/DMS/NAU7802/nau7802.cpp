#if defined(USE_NAU7802)

#include "nau7802.h"
#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

// ==================== I2C-PINS ====================
// ESP32-S3 hat keine festen I2C-Pins -> hier frei waehlbar.
// Default: SDA=GPIO8, SCL=GPIO9. Bei Bedarf anpassen.
#ifndef NAU7802_SDA
#define NAU7802_SDA 8
#endif
#ifndef NAU7802_SCL
#define NAU7802_SCL 9
#endif

static NAU7802 nau;

void nau7802_init() {
  Wire.begin(NAU7802_SDA, NAU7802_SCL);

  if (!nau.begin()) {
    Serial.println("FEHLER: NAU7802 nicht gefunden!");
    while (1);
  }

  nau.setGain(NAU7802_GAIN_128);
  nau.setSampleRate(NAU7802_SPS_80);
  nau.calibrateAFE();
}

void nau7802_tara() {
  nau.calculateZeroOffset();  // aktuellen Messwert als Nullpunkt speichern
  Serial.println("# Tara gesetzt");
}

void nau7802_update() {
  // Tara per Serial: 't' oder 'T' sendet einen neuen Nullpunkt
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') nau7802_tara();
  }

  if (nau.available()) {
    long rawValue = nau.getReading();
    float voltage_mV = (rawValue * 2400.0) / (8388607.0 * 128.0);

    // Teleplot-Format: ">name:wert" pro Zeile
    Serial.printf(">raw:%ld\n", rawValue);
    Serial.printf(">voltage_mV:%.4f\n", voltage_mV);
  }
}

#endif  // USE_NAU7802
