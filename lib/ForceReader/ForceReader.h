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

    float    taraUV_;
    bool     taraSet_;
    float    lastForce_;
    uint32_t startMs_;
};
