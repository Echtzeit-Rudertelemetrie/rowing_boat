#include "ad7124.h"
#include <SPI.h>

// ==================== PINS ====================
// DEINE VERDRAHTUNG (laut deiner Beschreibung)
#define PIN_CS    5
#define PIN_MOSI  23
#define PIN_MISO  19
#define PIN_SCK   18

// ==================== AD7124 REGISTERS ====================
#define REG_COMM         0x00
#define REG_ADC_CONTROL  0x01
#define REG_ADC_STATUS   0x02
#define REG_CH0          0x09
#define REG_CONFIG0      0x19
#define REG_FILTER0      0x29
#define REG_DATA         0x30

#define CMD_READ         0x80
#define CMD_WRITE        0x00
#define STATUS_RDY       0x80
#define CTRL_REF_EN     0x04
#define MODE_CONTINUOUS  0x02

#define CONFIG_BIPOLAR   0x10
#define CONFIG_REF_SEL   0x02
#define CONFIG_PGA(x)    (((x) & 0x07) << 8)

#define FILTER_FS(x)     ((x) & 0x7FF)
#define FILTER_FILT(x)   (((x) & 0x07) << 3)
#define FILTER_SINC4     0x04

#define CH_EN            0x80

static const float V_REF_uV = 2500000.0f;
static const float PGA = 128.0f;
static const float BITS = 8388608.0f;

volatile float ad7124_tara_uV = 0.0f;
volatile float ad7124_kraft = 0.0f;
volatile int ad7124_sample_count = 0;

#define AVG_SIZE 16
static float avg_buffer[AVG_SIZE];
static int avg_index = 0;
static float avg_sum = 0.0f;

// ==================== SOFTWARE SPI ====================

static void sw_spi_init() {
    pinMode(PIN_SCK, OUTPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_CS, OUTPUT);

    // CPOL=1: Clock ist HIGH im Ruhezustand
    digitalWrite(PIN_SCK, HIGH);
    digitalWrite(PIN_MOSI, HIGH);
    digitalWrite(PIN_CS, HIGH);
}

// Sende ein Byte und empfange gleichzeitig eines
// CPHA=1: Daten werden bei der steigenden Flanke gelesen
static uint8_t sw_spi_transfer(uint8_t tx) {
    uint8_t rx = 0;

    for (int i = 7; i >= 0; i--) {
        // Bit auf MOSI setzen
        digitalWrite(PIN_MOSI, (tx >> i) & 0x01);

        // Kurze Pause (halbe Clock-Periode bei 100kHz)
        delayMicroseconds(5);

        // Clock auf LOW (steigende Flanke kommt)
        digitalWrite(PIN_SCK, LOW);
        delayMicroseconds(5);

        // MISO lesen
        if (digitalRead(PIN_MISO)) {
            rx |= (1 << i);
        }

        // Clock zurueck auf HIGH
        digitalWrite(PIN_SCK, HIGH);
        delayMicroseconds(5);
    }

    return rx;
}

static void sw_cs_low() {
    digitalWrite(PIN_CS, LOW);
}

static void sw_cs_high() {
    digitalWrite(PIN_CS, HIGH);
}

static void sw_write_reg(uint8_t reg, const uint8_t *data, uint8_t len) {
    sw_cs_low();
    sw_spi_transfer(CMD_WRITE | (reg & 0x3F));
    for (uint8_t i = 0; i < len; i++) {
        sw_spi_transfer(data[i]);
    }
    sw_cs_high();
    delayMicroseconds(20);
}

static void sw_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    sw_cs_low();
    sw_spi_transfer(CMD_READ | (reg & 0x3F));
    for (uint8_t i = 0; i < len; i++) {
        data[i] = sw_spi_transfer(0x00);
    }
    sw_cs_high();
    delayMicroseconds(20);
}

// ==================== REGISTER TEST ====================

static void test_comm_register() {
    uint8_t d;

    Serial.println();
    Serial.println("=== Register Tests ===");

    // Test 1: Einfaches Byte lesen (kein CS, nur zum Debug)
    Serial.println();
    Serial.print("MISO direkt (ohne CS): ");
    Serial.println(digitalRead(PIN_MISO));

    // Test 2: COMM Register lesen
    d = 0xFF;
    sw_read_reg(REG_COMM, &d, 1);
    Serial.print("COMM: 0x");
    Serial.println(d, HEX);

    // Test 3: Nochmal lesen - diesmal mit Debug
    sw_cs_low();
    sw_spi_transfer(CMD_READ | REG_COMM);
    d = sw_spi_transfer(0x00);
    sw_cs_high();
    Serial.print("COMM (direkt): 0x");
    Serial.println(d, HEX);

    // Test 4: Einen Register-Schreibversuch mit falscher Adresse
    // Wenn das Geraet antwortet, bekommen wir irgendeinen Wert zurueck
    uint8_t test_data[3] = {0xAA, 0xBB, 0xCC};
    sw_write_reg(0xFF, test_data, 3);  // ungueltige Adresse
    delay(10);

    // Dann COMM lesen - wenn es 0xAA zurueckgibt, hat es was empfangen
    d = 0x00;
    sw_read_reg(REG_COMM, &d, 1);
    Serial.print("COMM nach Write(0xFF): 0x");
    Serial.println(d, HEX);

    // Test 5: ADC_STATUS lesen (sollte bei Betrieb etwas zurueckgeben)
    d = 0x00;
    sw_read_reg(REG_ADC_STATUS, &d, 1);
    Serial.print("ADC_STATUS: 0x");
    Serial.println(d, HEX);

    Serial.println();
}

void ad7124_init() {
    uint8_t d[3];

    Serial.println();
    Serial.println("=== AD7124 Debug Init ===");

    // 1. Pins initialisieren
    sw_spi_init();
    Serial.println("Pins: OK");

    // 2. SPI Reset senden (Datenblatt: 64 SCLK Zyklen mit DIN=1)
    sw_cs_low();
    for (int i = 0; i < 8; i++) sw_spi_transfer(0xFF);
    sw_cs_high();
    delay(50);
    Serial.println("Reset: OK");

    // 3. MISO Pegel pruefen (ohne CS)
    Serial.print("MISO (ohne CS, idle): ");
    Serial.println(digitalRead(PIN_MISO));

    // 4. Register Tests
    test_comm_register();

    // 5. Eigentliche Konfiguration
    Serial.println();
    Serial.println("=== Konfiguration ===");

    // ADC_CONTROL
    d[0] = 0x00; d[1] = CTRL_REF_EN; d[2] = 0x00;
    sw_write_reg(REG_ADC_CONTROL, d, 3);
    delay(10);
    d[0] = 0x00; sw_read_reg(REG_ADC_CONTROL, d, 3);
    Serial.print("ADC_CONTROL: 0x"); Serial.print(d[2], HEX); Serial.print(" "); Serial.print(d[1], HEX); Serial.print(" "); Serial.println(d[0], HEX);

    // CONFIG0
    d[0] = CONFIG_BIPOLAR | CONFIG_REF_SEL;
    d[1] = CONFIG_PGA(7);
    sw_write_reg(REG_CONFIG0, d, 2);
    delay(10);
    d[0] = 0x00; d[1] = 0x00; sw_read_reg(REG_CONFIG0, d, 2);
    Serial.print("CONFIG0: 0x"); Serial.print(d[1], HEX); Serial.print(" "); Serial.println(d[0], HEX);

    // FILTER0
    uint16_t filt = FILTER_SINC4 | FILTER_FILT(3) | FILTER_FS(19);
    d[0] = lowByte(filt); d[1] = highByte(filt);
    sw_write_reg(REG_FILTER0, d, 2);
    delay(10);
    d[0] = 0x00; d[1] = 0x00; sw_read_reg(REG_FILTER0, d, 2);
    Serial.print("FILTER0: 0x"); Serial.print(d[1], HEX); Serial.print(" "); Serial.println(d[0], HEX);

    // CH0
    d[0] = 0xA1; d[1] = CH_EN | 0x01;
    sw_write_reg(REG_CH0, d, 2);
    delay(10);
    d[0] = 0x00; d[1] = 0x00; sw_read_reg(REG_CH0, d, 2);
    Serial.print("CH0: 0x"); Serial.print(d[1], HEX); Serial.print(" "); Serial.println(d[0], HEX);

    Serial.println("=== Init Ende ===");
    Serial.println();
    Serial.println("zeit_ms,roh_dec,spannung_uV,kraft");

    for (int i = 0; i < AVG_SIZE; i++) avg_buffer[i] = 0.0f;
    avg_index = 0;
    avg_sum = 0.0f;
}

void ad7124_tara() {
    ad7124_tara_uV = 0.0f;
    Serial.println("Tara gesetzt!");
}

static float moving_avg(float val) {
    avg_sum -= avg_buffer[avg_index];
    avg_buffer[avg_index] = val;
    avg_sum += val;
    avg_index = (avg_index + 1) % AVG_SIZE;
    return avg_sum / AVG_SIZE;
}

void ad7124_update() {
    uint8_t st;
    sw_read_reg(REG_ADC_STATUS, &st, 1);

    if ((st & STATUS_RDY) != 0) return;

    uint8_t data[3];
    sw_read_reg(REG_DATA, data, 3);

    int32_t raw = ((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | ((int32_t)data[0]);
    if (raw & 0x800000) raw |= 0xFF000000;

    float uV = (float)raw * (V_REF_uV / (PGA * BITS));
    float avg = moving_avg(uV);

    ad7124_sample_count++;
    if (ad7124_tara_uV == 0.0f && ad7124_sample_count == 1) {
        ad7124_tara_uV = avg;
    }

    float kraft = (avg - ad7124_tara_uV) * 1.0f;
    ad7124_kraft = kraft;

    unsigned long t = millis();
    Serial.print(t);
    Serial.print(",");
    Serial.print(raw);
    Serial.print(",");
    Serial.print(avg, 2);
    Serial.print(",");
    Serial.println(kraft, 2);
}