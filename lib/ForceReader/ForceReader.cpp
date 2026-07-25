// ForceReader.cpp
#include "ForceReader.h"

// ==================== PINS / SPI ====================
// Seeed XIAO ESP32-S3. SPI-Pins laut Datenblatt (Pinout):
//   D8=SCK=GPIO7, D9=MISO=GPIO8, D10=MOSI=GPIO9. CS frei gewaehlt: D7=GPIO44.
// Auf dem ESP32-S3 existiert kein VSPI -> FSPI.
// ACHTUNG: GPIO19/20 sind auf dem S3 das native USB -> nicht als SPI belegen.
#define PIN_CS   44
#define PIN_MOSI 9
#define PIN_MISO 8
#define PIN_SCK  7

// ==================== AD7124 REGISTER ====================
#define REG_STATUS   0x00
#define REG_ADC_CTRL 0x01
#define REG_DATA     0x02
#define REG_ID       0x05
#define REG_CH0      0x09
#define REG_CH1      0x0A
#define REG_CONFIG0  0x19
#define REG_FILTER0  0x21

#define CMD_READ  0x40
#define CMD_WRITE 0x00

static const float VREF = 3.3f;
static const float PGA  = 128.0f;
static const float FS   = 8388608.0f;   // ADC-Vollausschlag (2^23)

// ==================== KALIBRIERUNG (getarte uV -> Newton) ====================
// PRO DMS EINMALIG mit bekannter Last bestimmen (beide DMS getrennt, da
// verschiedene Bruecken/Klebestellen leicht unterschiedlich empfindlich sind):
//   1. FORCE_READER_DEBUG unten aktiviert lassen, Firmware flashen.
//   2. Zelle OHNE Last booten, >2 s warten (Auto-Tara nullt) -> Teleplot
//      zeigt "tara0_uV" und "tara1_uV" ~ 0.
//   3. Bekannte Masse m anhaengen -> Kraft F = m * 9.81  (z.B. 1 kg -> 9.81 N).
//   4. Stabile Werte V0/V1 ("tara0_uV"/"tara1_uV", Mikrovolt, mit Vorzeichen)
//      ablesen.
//   5. UV_TO_N_CH0 = F / V0 und UV_TO_N_CH1 = F / V1 hier eintragen.
//   6. FORCE_READER_DEBUG auskommentieren, neu flashen -> Normalbetrieb.
// HINWEIS: Ist ein DMS gegensinnig aufgeklebt (Ausschlag mit umgekehrtem
// Vorzeichen), ergibt sich der jeweilige UV_TO_N automatisch negativ -> beide
// Kanaele zeigen dann trotzdem dieselbe (positive) Zugkraft.
static const float UV_TO_N_CH0 = 1.0f;   // TODO: DMS 0 mit bekannter Last kalibrieren!
static const float UV_TO_N_CH1 = 1.0f;   // TODO: DMS 1 mit bekannter Last kalibrieren!

// Teleplot output (>kraft_N / >tara?_uV) for bring-up/calibration. Production
// builds keep this off because USB-CDC writes can stall the 100-Hz event loop.
#ifndef FORCE_READER_DEBUG
#define FORCE_READER_DEBUG 0
#endif

float ForceReader::Channel::movingAvg(float v) {
    avgSum -= avgBuf[avgIdx];
    avgBuf[avgIdx] = v;
    avgSum += v;
    avgIdx = (avgIdx + 1) % AVG_SIZE;
    return avgSum / AVG_SIZE;
}

ForceReader::ForceReader()
: spi_(nullptr)
, ch_{}
, startMs_(0)
, combinedForce_(0)
, seqCh_(0) {
#if CONFIG_IDF_TARGET_ESP32S3
    spi_ = new SPIClass(FSPI);
#else
    spi_ = new SPIClass(VSPI);
#endif
    // Pro-Kanal-Zustand nullen und je DMS die eigene Kalibrierung setzen.
    for (uint8_t i = 0; i < NUM_CH; i++) {
        for (uint16_t k = 0; k < Channel::AVG_SIZE; k++) ch_[i].avgBuf[k] = 0.0f;
        ch_[i].avgIdx      = 0;
        ch_[i].avgSum      = 0.0f;
        ch_[i].taraUV      = 0.0f;
        ch_[i].taraSet     = false;
        ch_[i].lastForce   = 0.0f;
        ch_[i].rawUV       = 0.0f;
        ch_[i].taredUV     = 0.0f;
        ch_[i].nSamples    = 0;
        ch_[i].idleSinceMs = 0;
        ch_[i].idleTiming  = false;
        ch_[i].highSinceMs = 0;
        ch_[i].highTiming  = false;
    }
    ch_[0].uvToN = UV_TO_N_CH0;
    ch_[1].uvToN = UV_TO_N_CH1;
}

// ==================== SPI-HELFER ====================
void ForceReader::csLow()  { digitalWrite(PIN_CS, LOW); }
void ForceReader::csHigh() { digitalWrite(PIN_CS, HIGH); }

void ForceReader::writeReg(uint8_t addr, uint32_t data, uint8_t len) {
    csLow();
    spi_->transfer(CMD_WRITE | (addr & 0x3F));
    for (int i = len - 1; i >= 0; i--) {
        spi_->transfer((data >> (8 * i)) & 0xFF);
    }
    csHigh();
}

uint32_t ForceReader::readReg(uint8_t addr, uint8_t len) {
    csLow();
    spi_->transfer(CMD_READ | (addr & 0x3F));
    uint32_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        val = (val << 8) | spi_->transfer(0x00);
    }
    csHigh();
    return val;
}

void ForceReader::reset() {
    csLow();
    for (int i = 0; i < 8; i++) spi_->transfer(0xFF);
    csHigh();
    delay(10);
}

// ==================== OEFFENTLICHE API ====================
bool ForceReader::begin() {
    pinMode(PIN_CS, OUTPUT);
    csHigh();
    // ss = -1: CS rein manuell steuern. Sonst zieht die ESP-SPI-HW CS zwischen
    // den Bytes kurz auf HIGH -> AD7124 bricht den Frame ab und antwortet 0x00.
    spi_->begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
    spi_->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    reset();

    uint32_t id = readReg(REG_ID, 1);
    Serial.printf("AD7124 ID: 0x%02X\n", id);

    // WICHTIG: erst KONFIGURIEREN, dann Wandlung starten. Nach dem Reset laeuft
    // der AD7124 sofort in Continuous Conversion -> Kanal-/Config-Register erst
    // in STANDBY (MODE=2, Bits[5:2]=0b0010 -> 0x08) schreiben.
    //   0x0188 = REF_EN(0x100) | POWER_MODE=full(0x80) | MODE=standby(0x08)
    // KEIN DATA_STATUS: das an DATA angehaengte Byte ist auf diesem Chip NICHT der
    // Kanal-Tag (Messung: konstant 0x01), und auch das STATUS-Kanalfeld bleibt
    // konstant 0. Der ADC sequenziert die aktiven Kanaele aber deterministisch
    // (0,1,0,1,...) -> die Zuordnung erfolgt per Parity-Zaehler in sampleForce().
    writeReg(REG_ADC_CTRL, 0x0188, 2);

    // CONFIG0 = 0x087F: bipolar, AIN-Buffer an, REF_SEL=AVDD (ratiometrisch,
    // VREF=3,3 V -> Bruecken aus 3,3 V speisen!), PGA=128 (Bereich +/-25,8 mV).
    // Beide Kanaele teilen sich dieses Setup 0 (gleicher DMS-Typ/Bereich).
    writeReg(REG_CONFIG0, 0x087F, 2);
    // FILTER0 = 0x060080: Sinc4-Filter, FS=0x080 (vorher 0x180). Hoehere
    // Datenrate, damit bei zwei Kanaelen (jeder Kanalwechsel kostet volles
    // Sinc4-Einschwingen) genug Wandlungen pro Kanal fuer eine flotte Reaktion
    // ankommen. Gegen das dadurch etwas hoehere Rauschen mittelt AVG_SIZE.
    writeReg(REG_FILTER0, 0x060080, 3);
    // DMS 0 -> AIN0/AIN1 (0x8001), DMS 1 -> AIN2/AIN3 (0x8043); beide Setup 0.
    writeReg(REG_CH0, 0x8001, 2);
    writeReg(REG_CH1, 0x8043, 2);

    // Jetzt erst Continuous Conversion starten (MODE=0).
    //   0x0180 = REF_EN(0x100) | POWER_MODE=full(0x80), MODE=continuous(0)
    // Die erste Wandlung nach dem Start ist Kanal 0 -> Parity-Zaehler = 0.
    writeReg(REG_ADC_CTRL, 0x0180, 2);
    seqCh_ = 0;

    delay(100);
    startMs_ = millis();

#if FORCE_READER_DEBUG
    // Register zuruecklesen: steht Continuous (CTRL=0x0180) und sind BEIDE
    // Kanaele aktiv (CH0=0x8001, CH1=0x8043)? HINWEIS: der Readback verschluckt
    // beim Lesen das LSB (0x8001 liest 0x8000) -> ein reiner Lese-Artefakt, die
    // Writes selbst stimmen (beide Kanaele liefern ihr eigenes Differenzsignal).
    uint32_t ctrl = readReg(REG_ADC_CTRL, 2);
    uint32_t c0   = readReg(REG_CH0, 2);
    uint32_t c1   = readReg(REG_CH1, 2);
    Serial.printf("AD7124 cfg: CTRL=0x%04X CH0=0x%04X CH1=0x%04X\n", ctrl, c0, c1);
#endif

    // ID != 0x00/0xFF => der Chip antwortet ueber SPI.
    return (id != 0x00 && id != 0xFF);
}

void ForceReader::tara() {
    for (uint8_t i = 0; i < NUM_CH; i++) {
        ch_[i].taraUV  = ch_[i].avgSum / Channel::AVG_SIZE;
        ch_[i].taraSet = true;
    }
}

float ForceReader::forceChannel(uint8_t ch) const {
    return (ch < NUM_CH) ? ch_[ch].lastForce : 0.0f;
}

float ForceReader::drift() const {
    return ch_[0].lastForce - ch_[1].lastForce;
}

// Einen frisch gewandelten Rohwert (in uV) fuer Kanal idx einarbeiten:
// gleitender Mittelwert, Tara und Auto-Re-Tara gegen Drift -- alles pro DMS.
void ForceReader::processSample(uint8_t idx, float uV) {
    Channel& c = ch_[idx];
    c.nSamples++;

    float uV_avg = c.movingAvg(uV);
    c.rawUV = uV_avg;   // roher Wert vor Tara -> reagiert ungeschoent auf Kabel

    // Auto-Tara 2 s nach begin(), falls fuer diesen Kanal noch nicht gesetzt.
    if (!c.taraSet && (millis() - startMs_) > 2000) {
        c.taraUV  = uV_avg;
        c.taraSet = true;
    }

    c.taredUV   = uV_avg - c.taraUV;
    c.lastForce = c.taredUV * c.uvToN;   // Newton

    // Auto-Re-Tara gegen Drift: Nullpunkt nur nachziehen, wenn die Kraft
    // laenger ununterbrochen im Drift-Zustand bleibt (nicht bei jedem
    // einzelnen Ausschlag -> sonst Ratschen des Nullpunkts). Erst ab
    // gesetztem Tara aktiv, damit die Auto-Tara nach begin() nicht stoert.
    // Zwei unabhaengige Faelle mit eigenem Timer:
    //   a) Ruhe/Negativdrift: alles UNTER der positiven Zug-Schwelle.
    //   b) Hoch-Drift: alles UEBER AUTO_TARA_MAX_N (jenseits des App-Bereichs).
    bool retara = false;

    if (c.taraSet && c.lastForce < AUTO_TARA_BAND_N) {
        if (!c.idleTiming) {
            c.idleTiming  = true;
            c.idleSinceMs = millis();
        } else if (millis() - c.idleSinceMs >= AUTO_TARA_HOLD_MS) {
            retara = true;
        }
    } else {
        c.idleTiming = false;       // Zug/Ausreisser -> Ruhephase abgebrochen
    }

    if (c.taraSet && c.lastForce > AUTO_TARA_MAX_N) {
        if (!c.highTiming) {
            c.highTiming  = true;
            c.highSinceMs = millis();
        } else if (millis() - c.highSinceMs >= AUTO_TARA_HIGH_HOLD_MS) {
            retara = true;
        }
    } else {
        c.highTiming = false;       // wieder im Bereich -> Hoch-Drift abgebrochen
    }

    if (retara) {
        c.taraUV     = uV_avg;      // Nullpunkt auf aktuellen Mittelwert
        c.lastForce  = 0.0f;
        c.idleTiming = false;       // beide Timer neu starten
        c.highTiming = false;
    }
}

float ForceReader::sampleForce() {
    uint32_t status = readReg(REG_STATUS, 1);
    if (status & 0x80) {          // /RDY: Wandlung noch nicht fertig
        return combinedForce_;    // letzten gueltigen Wert weiterreichen
    }

    // 3 Datenbytes lesen. Der ADC sequenziert die aktiven Kanaele fest der Reihe
    // nach (0,1,0,1,...); da kein brauchbarer Hardware-Kanal-Tag existiert, wird
    // die Zuordnung ueber den Parity-Zaehler seqCh_ mitgefuehrt. Das haelt, solange
    // keine Wandlung verpasst wird -> der Event-Loop pollt schneller (~100 Hz) als
    // der ADC neue Werte liefert (~20-40 ms je Wandlung).
    uint32_t data  = readReg(REG_DATA, 3);
    int32_t  raw   = (int32_t)data - 0x800000;

    float uV = (raw * VREF * 1e6f) / (PGA * FS);

    uint8_t chTag = seqCh_;
    seqCh_ = (seqCh_ + 1) % NUM_CH;

    processSample(chTag, uV);

    // Beide DMS messen dieselbe Kraft am selben Metallstueck -> Mittelung senkt
    // das Rauschen (~sqrt(2)). Solange ein Kanal noch keinen Wert hatte, ist
    // dessen lastForce 0; nach wenigen Wandlungen sind beide gefuellt.
    combinedForce_ = 0.5f * (ch_[0].lastForce + ch_[1].lastForce);

#if FORCE_READER_DEBUG
    // Teleplot (">name:value"): kombinierte Kraft, Einzelkraefte, Drift und die
    // getarten uV je Kanal (tara?_uV zur Kalibrierung). Bewusst nur ~10 Hz und
    // wenige Signale: USB-CDC-Writes bremsen sonst den Event-Loop und lassen
    // Teleplot laggen.
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg >= 100) {
        lastDbg = millis();
        Serial.printf(">kraft_N:%.3f\n>kraft0_N:%.3f\n>kraft1_N:%.3f\n>drift_N:%.3f\n",
                      combinedForce_, ch_[0].lastForce, ch_[1].lastForce, drift());
    }
#endif

    return combinedForce_;
}
