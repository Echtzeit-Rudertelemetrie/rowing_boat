#pragma once

#include <Arduino.h>

// DMS-Kraftmessung ueber AD7124-8 (24-Bit Sigma-Delta ADC, Hardware-SPI).
// Verdrahtung siehe ad7124_wiring.md.
//
// Aktiv nur, wenn beim Build -D USE_AD7124 gesetzt ist (siehe platformio.ini).

void ad7124_init();    // ADC zuruecksetzen, konfigurieren, Header ausgeben
void ad7124_update();  // einen Messwert lesen, mitteln, als Teleplot ausgeben
void ad7124_tara();    // aktuellen Mittelwert als Nullpunkt setzen
