#pragma once

#include <Arduino.h>

// DMS-Kraftmessung ueber NAU7802 (24-Bit Waegezellen-Verstaerker, I2C).
// Gedacht fuer Waveshare ESP32-S3-Zero (I2C-Pins frei waehlbar).
//
// Aktiv nur, wenn beim Build -D USE_NAU7802 gesetzt ist (siehe platformio.ini).

void nau7802_init();    // I2C starten, NAU7802 konfigurieren, kalibrieren
void nau7802_update();  // einen Messwert lesen und ausgeben
void nau7802_tara();    // aktuellen Wert als Nullpunkt (Offset) setzen
