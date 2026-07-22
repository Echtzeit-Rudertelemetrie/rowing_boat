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
#define REG_CONFIG0  0x19
#define REG_FILTER0  0x21

#define CMD_READ  0x40
#define CMD_WRITE 0x00

static const float VREF = 3.3f;
static const float PGA  = 128.0f;
static const float FS   = 8388608.0f;   // ADC-Vollausschlag (2^23)

// ==================== KALIBRIERUNG (getarte uV -> Newton) ====================
// EINMALIG mit bekannter Last bestimmen:
//   1. FORCE_READER_DEBUG unten aktiviert lassen, Firmware flashen.
//   2. Zelle OHNE Last booten, >2 s warten (Auto-Tara nullt) -> Teleplot
//      zeigt "tara_uV" ~ 0.
//   3. Bekannte Masse m anhaengen -> Kraft F = m * 9.81  (z.B. 1 kg -> 9.81 N).
//   4. Stabilen "tara_uV"-Wert V ablesen (Mikrovolt, mit Vorzeichen).
//   5. UV_TO_N = F / V  hier eintragen (Newton pro Mikrovolt).
//   6. FORCE_READER_DEBUG auskommentieren, neu flashen -> Normalbetrieb.
static const float UV_TO_N = 1.0f;   // TODO: mit bekannter Last kalibrieren!

// Teleplot-Ausgabe (>kraft_N / >tara_uV) fuer Bring-up/Kalibrierung.
// Fuer den Normalbetrieb auskommentieren.
#define FORCE_READER_DEBUG

ForceReader::ForceReader()
: spi_(nullptr)
, avgBuf_{0}
, avgIdx_(0)
, avgSum_(0)
, taraUV_(0)
, taraSet_(false)
, lastForce_(0)
, startMs_(0) {
#if CONFIG_IDF_TARGET_ESP32S3
    spi_ = new SPIClass(FSPI);
#else
    spi_ = new SPIClass(VSPI);
#endif
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

float ForceReader::movingAvg(float v) {
    avgSum_ -= avgBuf_[avgIdx_];
    avgBuf_[avgIdx_] = v;
    avgSum_ += v;
    avgIdx_ = (avgIdx_ + 1) % AVG_SIZE;
    return avgSum_ / AVG_SIZE;
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

    writeReg(REG_ADC_CTRL, 0x0180, 2);
    // CONFIG0 = 0x087F: bipolar, AIN-Buffer an, REF_SEL=AVDD (ratiometrisch,
    // VREF=3,3 V -> Bruecke aus 3,3 V speisen!), PGA=128 (Bereich +/-25,8 mV).
    writeReg(REG_CONFIG0, 0x087F, 2);
    // FILTER0 = 0x060180: Sinc4-Filter, FS=0x180 -> ~50 SPS.
    writeReg(REG_FILTER0, 0x060180, 3);
    writeReg(REG_CH0, 0x8001, 2);

    delay(100);
    startMs_ = millis();

    // ID != 0x00/0xFF => der Chip antwortet ueber SPI.
    return (id != 0x00 && id != 0xFF);
}

void ForceReader::tara() {
    taraUV_  = avgSum_ / AVG_SIZE;
    taraSet_ = true;
}

float ForceReader::sampleForce() {
    uint32_t status = readReg(REG_STATUS, 1);
    if (status & 0x80) {          // /RDY: Wandlung noch nicht fertig
        return lastForce_;        // letzten gueltigen Wert weiterreichen
    }

    uint32_t data = readReg(REG_DATA, 3);
    int32_t  raw  = (int32_t)data - 0x800000;

    float uV     = (raw * VREF * 1e6f) / (PGA * FS);
    float uV_avg = movingAvg(uV);

    // Auto-Tara 2 s nach begin(), falls noch nicht gesetzt.
    if (!taraSet_ && (millis() - startMs_) > 2000) {
        taraUV_  = uV_avg;
        taraSet_ = true;
    }

    float tared_uV = uV_avg - taraUV_;
    lastForce_ = tared_uV * UV_TO_N;   // Newton

#ifdef FORCE_READER_DEBUG
    // Teleplot (">name:value"): Kraft in N + getarte uV (uV auch zur Kalibrierung).
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg >= 50) {
        lastDbg = millis();
        Serial.printf(">kraft_N:%.3f\n>tara_uV:%.2f\n", lastForce_, tared_uV);
    }
#endif

    return lastForce_;
}
