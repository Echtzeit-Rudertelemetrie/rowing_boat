#pragma once

#include <Arduino.h>
#include <SPI.h>

// DMS-Kraftmessung ueber den AD7124-8 (24-Bit Sigma-Delta ADC, Hardware-SPI).
// Board: Seeed XIAO ESP32-S3. Referenz ist ratiometrisch = AVDD (3,3 V) -> die
// DMS-Bruecke MUSS aus denselben 3,3 V gespeist werden.
//
// SPI-Mode 3 (CPOL=1, CPHA=1), 1 MHz. CS wird manuell gesteuert (begin(..,-1)),
// sonst zieht die ESP-SPI-HW CS zwischen den Bytes kurz auf HIGH und der AD7124
// bricht den Frame ab und antwortet 0x00.
class ForceReader {
public:
    ForceReader();

    bool  begin();         // SPI + AD7124 zuruecksetzen und konfigurieren
    float sampleForce();   // einen (nicht-blockierenden) Messwert lesen, Kraft in Newton
    void  tara();          // aktuellen Mittelwert als Nullpunkt setzen

private:
    void     csLow();
    void     csHigh();
    void     writeReg(uint8_t addr, uint32_t data, uint8_t len);
    uint32_t readReg(uint8_t addr, uint8_t len);
    void     reset();
    float    movingAvg(float v);

    SPIClass* spi_;

    static constexpr uint16_t AVG_SIZE = 16;
    float   avgBuf_[AVG_SIZE];
    uint8_t avgIdx_;
    float   avgSum_;

    // Auto-Re-Tara gegen langsame DMS-Drift: Bleibt die (getarte) Kraft
    // laenger als AUTO_TARA_HOLD_MS ununterbrochen UNTER der Zug-Schwelle
    // AUTO_TARA_BAND_N (einseitig -> beliebig negativ zaehlt als Ruhe), wird
    // der Nullpunkt auf den aktuellen Mittelwert nachgezogen. Nur ein echter
    // Ruderzug (deutlich positiv, > Schwelle) setzt den Timer zurueck.
    // ACHTUNG: Solange UV_TO_N noch nicht kalibriert ist (== 1.0), ist "N"
    // hier effektiv Mikrovolt -> die Schwelle knapp ueber das positive
    // Ruherauschen legen und nach der Kalibrierung neu einstellen.
    static constexpr float    AUTO_TARA_BAND_N = 0.5f;   // positive Zug-Schwelle
    static constexpr uint32_t AUTO_TARA_HOLD_MS = 1000;  // wie lange im Band

    // Hoch-Drift: bleibt die Kraft laenger als AUTO_TARA_HIGH_HOLD_MS ueber
    // AUTO_TARA_MAX_N (Obergrenze der App-Anzeige), wird ebenfalls neu getart.
    // Haltezeit bewusst laenger als ein echter Zug > MAX dauern kann.
    static constexpr float    AUTO_TARA_MAX_N = 1000.0f;      // App-Obergrenze
    static constexpr uint32_t AUTO_TARA_HIGH_HOLD_MS = 1500;  // wie lange drueber

    float    taraUV_;
    bool     taraSet_;
    float    lastForce_;
    uint32_t startMs_;
    uint32_t idleSinceMs_;   // Beginn der aktuellen Ruhephase (0 = keine)
    bool     idleTiming_;    // laeuft gerade eine Ruhephase?
    uint32_t highSinceMs_;   // Beginn der aktuellen Hoch-Drift-Phase
    bool     highTiming_;    // laeuft gerade eine Hoch-Drift-Phase?
};
