#pragma once

#include <Arduino.h>
#include <SPI.h>

// DMS-Kraftmessung ueber den AD7124-8 (24-Bit Sigma-Delta ADC, Hardware-SPI).
// Board: Seeed XIAO ESP32-S3. Referenz ist ratiometrisch = AVDD (3,3 V) -> die
// DMS-Bruecken MUESSEN aus denselben 3,3 V gespeist werden.
//
// ZWEI DMS auf demselben Metallstueck, jeder in einer eigenen Viertelbruecke:
//   DMS 0 (CH0) -> differentiell AIN0/AIN2
//   DMS 1 (CH1) -> differentiell AIN4/AIN6
// Der AD7124 sequenziert im Continuous-Mode automatisch durch beide aktiven
// Kanaele (0,1,0,1,...). ACHTUNG: auf diesem Chip liefert KEIN Hardware-Tag die
// Kanalnummer (weder STATUS-Kanalfeld noch DATA_STATUS-Byte, beide konstant) ->
// die Zuordnung laeuft ueber einen Parity-Zaehler, siehe sampleForce().
// Beide Kanaele werden getrennt gemittelt/getart (jeder DMS driftet fuer sich)
// und dann zu EINEM Kraftwert gemittelt -> weniger Rauschen. Die Differenz der
// beiden (drift()) macht eine auseinanderlaufende Drift sichtbar.
//
// SPI-Mode 3 (CPOL=1, CPHA=1), 1 MHz. CS wird manuell gesteuert (begin(..,-1)),
// sonst zieht die ESP-SPI-HW CS zwischen den Bytes kurz auf HIGH und der AD7124
// bricht den Frame ab und antwortet 0x00.
class ForceReader {
public:
    ForceReader();

    bool  begin();         // SPI + AD7124 zuruecksetzen und konfigurieren
    float sampleForce();   // einen (nicht-blockierenden) Messwert lesen: Mittel
                           // beider DMS, Kraft in Newton
    void  tara();          // beide Kanaele auf ihren aktuellen Mittelwert nullen

    // Zusatz fuer Analyse/Debug (keine Wirkung auf die Telemetrie):
    static constexpr uint8_t NUM_CH = 2;
    float forceChannel(uint8_t ch) const;  // Einzelkraft von DMS 0 bzw. 1 [N]
    float drift() const;                    // ch0 - ch1, sollte ~0 sein [N]

private:
    void     csLow();
    void     csHigh();
    void     writeReg(uint8_t addr, uint32_t data, uint8_t len);
    uint32_t readReg(uint8_t addr, uint8_t len);
    void     reset();
    void     processSample(uint8_t idx, float uV);  // frischen Wert einarbeiten

    // Pro-Kanal-Zustand: eigener gleitender Mittelwert, Nullpunkt und
    // Auto-Re-Tara-Timer, da beide DMS unabhaengig voneinander driften.
    struct Channel {
        // Kleiner als frueher (war 16): bei zwei Kanaelen ist die Rate PRO Kanal
        // wegen des Sinc4-Neueinschwingens je Kanalwechsel viel niedriger, ein
        // 16er-Mittel wuerde ~3 s ueberstreichen -> traege. Zusammen mit der
        // hoeheren ADC-Datenrate (FILTER0) ergibt 6 wieder ~0,3-0,4 s.
        static constexpr uint16_t AVG_SIZE = 6;
        float    avgBuf[AVG_SIZE];
        uint8_t  avgIdx;
        float    avgSum;

        float    uvToN;        // Kalibrierung: Newton pro Mikrovolt (je DMS eigen)
        float    taraUV;
        bool     taraSet;
        float    lastForce;    // zuletzt berechnete Kraft dieses DMS [N]
        float    rawUV;        // gemitteltes uV VOR Tara (roher Diagnosewert)
        float    taredUV;      // zuletzt getarte uV (fuer Debug/Kalibrierung)
        uint32_t nSamples;     // wie viele Wandlungen kamen fuer diesen Kanal an

        uint32_t idleSinceMs;  // Beginn der aktuellen Ruhephase (0 = keine)
        bool     idleTiming;   // laeuft gerade eine Ruhephase?
        uint32_t highSinceMs;  // Beginn der aktuellen Hoch-Drift-Phase
        bool     highTiming;   // laeuft gerade eine Hoch-Drift-Phase?

        float    movingAvg(float v);
    };

    // Auto-Re-Tara gegen langsame DMS-Drift: Bleibt die (getarte) Kraft eines
    // Kanals laenger als AUTO_TARA_HOLD_MS ununterbrochen UNTER der Zug-Schwelle
    // AUTO_TARA_BAND_N (einseitig -> beliebig negativ zaehlt als Ruhe), wird
    // dessen Nullpunkt auf den aktuellen Mittelwert nachgezogen. Nur ein echter
    // Ruderzug (deutlich positiv, > Schwelle) setzt den Timer zurueck.
    // ACHTUNG: Solange uvToN noch nicht kalibriert ist (== 1.0), ist "N" hier
    // effektiv Mikrovolt -> die Schwelle knapp ueber das positive Ruherauschen
    // legen und nach der Kalibrierung neu einstellen.
    static constexpr float    AUTO_TARA_BAND_N = 0.5f;   // positive Zug-Schwelle
    static constexpr uint32_t AUTO_TARA_HOLD_MS = 1000;  // wie lange im Band

    // Hoch-Drift: bleibt die Kraft laenger als AUTO_TARA_HIGH_HOLD_MS ueber
    // AUTO_TARA_MAX_N (Obergrenze der App-Anzeige), wird ebenfalls neu getart.
    // Haltezeit bewusst laenger als ein echter Zug > MAX dauern kann.
    static constexpr float    AUTO_TARA_MAX_N = 1000.0f;      // App-Obergrenze
    static constexpr uint32_t AUTO_TARA_HIGH_HOLD_MS = 1500;  // wie lange drueber

    SPIClass* spi_;
    Channel   ch_[NUM_CH];
    uint32_t  startMs_;
    float     combinedForce_;   // gemittelte Kraft beider DMS [N]
    uint8_t   seqCh_;           // naechster erwarteter Kanal (Parity-Zaehler)
};
