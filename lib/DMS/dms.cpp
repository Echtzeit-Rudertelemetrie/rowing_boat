#include "dms.h"

static NAU7802 nau;
static unsigned long startTime;

void dms_init() {
    Wire.begin();

    if (!nau.begin()) {
        Serial.println("FEHLER: NAU7802 nicht gefunden!");
        while (1);
    }

    nau.setGain(NAU7802_GAIN_128);
    nau.setSampleRate(NAU7802_SPS_80);
    nau.calibrateAFE();

    Serial.println("zeit_ms,raw,spannung_mV");
    startTime = millis();
}

void dms_update() {
    if (nau.available()) {
        long rawValue = nau.getReading();
        float voltage_mV = (rawValue * 2400.0) / (8388607.0 * 128.0);
        unsigned long zeit = millis() - startTime;

        Serial.print(zeit);
        Serial.print(",");
        Serial.print(rawValue);
        Serial.print(",");
        Serial.println(voltage_mV, 4);
    }
    delay(100);
}