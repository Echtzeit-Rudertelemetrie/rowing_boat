#if defined(USE_AD7124)

#include "ad7124.h"
#include <SPI.h>

// ==================== PINS / SPI (ESP32-S3-Zero) ====================
// Achtung: GPIO19/20 sind beim S3 das native USB -> nicht belegen!
// VSPI existiert auf dem S3 nicht, daher FSPI.
#define PIN_CS   10
#define PIN_MOSI 11
#define PIN_SCK  12
#define PIN_MISO 13

static SPIClass *spi = new SPIClass(FSPI);

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
static const float FS   = 8388608.0f;
static const uint16_t AVG_SIZE = 16;

static float tara_uV = 0;
static float avg_buf[AVG_SIZE] = {0};
static uint8_t avg_idx = 0;
static float avg_sum = 0;
static bool tara_set = false;

// ==================== SPI-HELFER ====================
static void csLow()  { digitalWrite(PIN_CS, LOW); }
static void csHigh() { digitalWrite(PIN_CS, HIGH); }

static void writeReg(uint8_t addr, uint32_t data, uint8_t len) {
  csLow();
  spi->transfer(CMD_WRITE | (addr & 0x3F));
  for (int i = len - 1; i >= 0; i--) {
    spi->transfer((data >> (8 * i)) & 0xFF);
  }
  csHigh();
}

static uint32_t readReg(uint8_t addr, uint8_t len) {
  csLow();
  spi->transfer(CMD_READ | (addr & 0x3F));
  uint32_t val = 0;
  for (uint8_t i = 0; i < len; i++) {
    val = (val << 8) | spi->transfer(0x00);
  }
  csHigh();
  return val;
}

static void ad7124_reset() {
  csLow();
  for (int i = 0; i < 8; i++) spi->transfer(0xFF);
  csHigh();
  delay(10);
}

static float movingAvg(float v) {
  avg_sum -= avg_buf[avg_idx];
  avg_buf[avg_idx] = v;
  avg_sum += v;
  avg_idx = (avg_idx + 1) % AVG_SIZE;
  return avg_sum / AVG_SIZE;
}

// ==================== OEFFENTLICHE API ====================
void ad7124_init() {
  pinMode(PIN_CS, OUTPUT);
  csHigh();
  spi->begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  ad7124_reset();

  uint32_t id = readReg(REG_ID, 1);
  Serial.printf("AD7124 ID: 0x%02X\n", id);

  writeReg(REG_ADC_CTRL, 0x0180, 2);
  writeReg(REG_CONFIG0, 0x087F, 2);
  writeReg(REG_FILTER0, 0x060180, 3);
  writeReg(REG_CH0, 0x8001, 2);

  delay(100);
  Serial.println("zeit_ms,raw,spannung_uV,kraft");
}

void ad7124_tara() {
  tara_uV = avg_sum / AVG_SIZE;
  tara_set = true;
  Serial.println("# Tara gesetzt");
}

void ad7124_update() {
  // Tara per Serial: 't' oder 'T' sendet einen neuen Nullpunkt
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') ad7124_tara();
  }

  uint32_t status = readReg(REG_STATUS, 1);
  if (status & 0x80) {  // /RDY: Wandlung noch nicht fertig
    delay(5);
    return;
  }

  uint32_t data = readReg(REG_DATA, 3);
  int32_t raw = (int32_t)data - 0x800000;

  float uV = (raw * VREF * 1e6) / (PGA * FS);
  float uV_avg = movingAvg(uV);

  // Auto-Tara nach 2 s, falls noch nicht gesetzt
  if (!tara_set && millis() > 2000) {
    tara_uV = uV_avg;
    tara_set = true;
  }

  float kraft = (uV_avg - tara_uV) * 1.0f;  // TODO: Kalibrierfaktor -> N

  static uint32_t last = 0;
  if (millis() - last >= 100) {
    last = millis();
    Serial.printf(">last:%lu\n", last);
    Serial.printf(">raw:%ld\n", raw);
    Serial.printf(">uV_avg:%.2f\n", uV_avg);
    Serial.printf(">kraft:%.2f\n", kraft);
  }
}

#endif  // USE_AD7124
