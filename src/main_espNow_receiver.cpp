<<<<<<< HEAD
/***
 * ESP-NOW receiver / aggregator (ESP32 nodemcu-32s, env: prod_aggregator)
 *
 * Role in the boat:
 *   Receives RowingPackets from 2-8 oarlock senders via ESP-NOW (lib/ESPNOW),
 *   then forwards the aggregated data over UART (TX) to the BLE-sender MCU.
 *
 * Status (2026-06-17):
 *   The code below is still a per-oarlock angle + DMS measurement loop
 *   (gyro-EKF by default, Hall via USE_HALL_SENSOR) printing to Serial/teleplot
 *   (>dms/..., >as5600/...). It does NOT yet receive over ESP-NOW or forward
 *   over UART.
 *   TODO  - espnow_init_receiver(), unpack RowingPacket in onDataRecv(), UART
 *           forward to the BLE sender. (This measurement loop most likely
 *           belongs in main_espNow_sender.cpp.)
 */

#include <Arduino.h>
#include <SPI.h>

// ─────────────────────────────────────────────────────────────────────────────
// prod_32mini: Winkelmessung + DMS
//
// Winkelquelle umschalten:
//   0 = Gyro-EKF (MPU6050 + MMC5603 Magnetometer, I2C)
//   1 = Hall-Sensor (AS5600, I2C)
// ─────────────────────────────────────────────────────────────────────────────
#define USE_HALL_SENSOR 0

#if USE_HALL_SENSOR
#include <Wire.h>
#include "hall_angle.h"
static HallAngle hallSensor;
#else
#include "gyro.h"
#endif

// ─────────────────────────────────────────────────────────────────────────────
// AD7124-8 (DMS) — Hardware-SPI, identisch zu src/main.cpp
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_CS 5
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK 18

#if CONFIG_IDF_TARGET_ESP32S3 //mini
  SPIClass *spi = new SPIClass(FSPI);
#else
  SPIClass *spi = new SPIClass(VSPI);
#endif

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
}

float movingAvg(float v) {
  avg_sum -= avg_buf[avg_idx];
  avg_buf[avg_idx] = v;
  avg_sum += v;
  avg_idx = (avg_idx + 1) % AVG_SIZE;
  return avg_sum / AVG_SIZE;
}

void dmsUpdate() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='t' || c=='T') {
      tara_uV = movingAvg(0);
      tara_set = true;
      Serial.println("# Tara gesetzt");
    }
  }

  uint32_t status = readReg(REG_STATUS, 1);
  if (status & 0x80) return; // kein neues Sample bereit

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
    Serial.printf(">dms/spannung_uV: %.2f\n", uV_avg);
    Serial.printf(">dms/kraft: %.2f\n", kraft);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Winkelmessung
// ─────────────────────────────────────────────────────────────────────────────
void angleUpdate() {
#if USE_HALL_SENSOR
  if (hallSensor.update()) {
    Serial.printf(">as5600/angle_raw: %.2f\n",       hallSensor.getAngleRaw());
    Serial.printf(">as5600/angle_with_calc: %.2f\n", hallSensor.getAngleCalc());
  }
#else
  sampleAndPlot(0.0f);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== prod_32mini: Winkel + DMS ===");

#if USE_HALL_SENSOR
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  hallSensor.begin();
#else
  setupGyro();
  calibrateGyro();
  delay(100);
#endif

  ad7124_init();
}

void loop() {
  angleUpdate();
  dmsUpdate();
}
=======
#include <Arduino.h>
#include "AppTypes.h"
#include "EspNowReceiver.h"
#include "UartForwarder.h"

// Instanzen unserer Klassen
EspNowReceiver espReceiver;
UartForwarder uartBridge;

void setup() {
    // USB-Konsole für Debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== ESP32 ESP-NOW -> UART Bridge ===");

    // UART-Verbindung zum XIAO starten
    uartBridge.begin();

    // ESP-NOW Empfänger starten
    if (!espReceiver.begin()) {
        Serial.println("Kritischer Fehler bei ESP-NOW Initialisierung. Neustart...");
        delay(3000);
        ESP.restart();
    }
}

void loop() {
    MeasurementPack incomingPack;

    // 1. Prüfen, ob ein neues Paket via ESP-NOW reingekommen ist
    if (espReceiver.getPacket(incomingPack)) {
        
        // --- Debug Ausgabe auf USB ---
        const uint8_t espId = static_cast<uint8_t>((incomingPack.espIdAndSeqenceNum >> 29) & 0x07u);
        const uint32_t seq  = incomingPack.espIdAndSeqenceNum & 0x1FFFFFFFu;
        Serial.printf("[RX] ESP-ID: %u | Seq: %lu -> Leite via UART weiter...\n", espId, static_cast<unsigned long>(seq));

        // 2. Paket unverändert via UART2 an den XIAO weiterleiten
        uartBridge.forwardPacket(incomingPack);
    }
    else {
        delay(45);
        for(int i = 0; i < PACKET_VALUES; i++)
        {
            incomingPack.angle_values[i] = (uint16_t) i;
            incomingPack.force_values[i] = (uint16_t) i;
        }
        incomingPack.espIdAndSeqenceNum = (uint32_t) 764874;

        uartBridge.forwardPacket(incomingPack);
    }

    // 3. (Optional) Schauen, ob der XIAO etwas zurückgesendet hat (Diagnose)
    uartBridge.checkIncoming();

    // Kurzes Delay um den Watchdog nicht zu triggern
    delay(1); 
}
>>>>>>> alles_in_projekten
