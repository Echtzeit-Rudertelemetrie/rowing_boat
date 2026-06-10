#include <Arduino.h>
#include <SPI.h>

#define PIN_CS 5
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18

SPIClass *spi = new SPIClass(VSPI);

#define REG_STATUS 0x00
#define REG_ADC_CTRL 0x01
#define REG_DATA 0x02
#define REG_ID 0x05
#define REG_CH0 0x09
#define REG_CONFIG0 0x19
#define REG_FILTER0 0x21

#define CMD_READ 0x40
#define CMD_WRITE 0x00

const float VREF = 3.3f;
const float PGA = 128.0f;
const float FS = 8388608.0f;
const uint16_t AVG_SIZE = 16;

float tara_uV = 0;
float avg_buf[16] = {0};
uint8_t avg_idx = 0;
float avg_sum = 0;
bool tara_set = false;

void csLow() { digitalWrite(PIN_CS, LOW); }
void csHigh() { digitalWrite(PIN_CS, HIGH); }

void writeReg(uint8_t addr, uint32_t data, uint8_t len) {
  csLow();
  spi->transfer(CMD_WRITE | (addr & 0x3F));
  for (int i = len-1; i >= 0; i--) {
    spi->transfer((data >> (8*i)) & 0xFF);
  }
  csHigh();
}

uint32_t readReg(uint8_t addr, uint8_t len) {
  csLow();
  spi->transfer(CMD_READ | (addr & 0x3F));
  uint32_t val = 0;
  for (uint8_t i = 0; i < len; i++) {
    val = (val << 8) | spi->transfer(0x00);
  }
  csHigh();
  return val;
}

void ad7124_reset() {
  csLow();
  for (int i=0;i<8;i++) spi->transfer(0xFF);
  csHigh();
  delay(10);
}

void ad7124_init() {
  pinMode(PIN_CS, OUTPUT);
  csHigh();
  spi->begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  ad7124_reset();

  uint32_t id = readReg(REG_ID, 1);
  Serial.printf("AD7124 ID: 0x%02X\n", id);

  writeReg(REG_ADC_CTRL, 0x0180, 2);

  uint16_t cfg0 = 0x087F;
  writeReg(REG_CONFIG0, cfg0, 2);

  writeReg(REG_FILTER0, 0x060180, 3);

  uint16_t ch0 = 0x8001;
  writeReg(REG_CH0, ch0, 2);

  delay(100);
  Serial.println("zeit_ms,raw,spannung_uV,kraft");
}

float movingAvg(float v) {
  avg_sum -= avg_buf[avg_idx];
  avg_buf[avg_idx] = v;
  avg_sum += v;
  avg_idx = (avg_idx + 1) % AVG_SIZE;
  return avg_sum / AVG_SIZE;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== AD7124-8 DMS Messung ===");
  ad7124_init();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='t' || c=='T') {
      tara_uV = movingAvg(0);
      tara_set = true;
      Serial.println("# Tara gesetzt");
    }
  }

  uint32_t status = readReg(REG_STATUS, 1);
  if (status & 0x80) {
    delay(5);
    return;
  }

  uint32_t data = readReg(REG_DATA, 3);
  int32_t raw = (int32_t)data - 0x800000;

  float uV = (raw * VREF * 1e6) / (PGA * FS);
  float uV_avg = movingAvg(uV);

  if (!tara_set && millis() > 2000) {
    tara_uV = uV_avg;
    tara_set = true;
  }

  float kraft = (uV_avg - tara_uV) * 1.0f;

  static uint32_t last = 0;
  if (millis() - last >= 100) {
    last = millis();
    //Serial.printf("%lu,%ld,%.2f,%.2f\n", last, raw, uV_avg, kraft);
    Serial.printf(">last:%lu\n", last);
    Serial.printf(">raw:%ld\n", raw);
    Serial.printf(">uV_avg:%.2f\n", uV_avg);
    Serial.printf(">kraft:%.2f\n", kraft);
  }
}