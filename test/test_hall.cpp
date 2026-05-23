#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "hall_angle.h"

// ─────────────────────────────────────────────────────────────
static HallAngle sensor;

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    esp_bt_controller_deinit();
    Serial.println("Wi-Fi and Bluetooth disabled.");

    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    sensor.begin();
}

// ─────────────────────────────────────────────────────────────
void loop() {
    if (!sensor.update()) return;

    Serial.printf(">as5600/angle_raw: %.2f\n",       sensor.getAngleRaw());
    Serial.printf(">as5600/angle_with_calc: %.2f\n", sensor.getAngleCalc());
    Serial.printf(">as5600/magnitude: %d\n",         sensor.getMagnitude());
    Serial.printf(">as5600/agc: %d\n",               sensor.getAGC());
}

/*#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"

#define AS5600_ADDR        0x36
#define REG_ANGLE_H        0x0E
#define SAMPLE_INTERVAL_US 10000  // 100 Hz
#define EMA_ALPHA          0.1f
#define AGC_REG            0x1A

// ─────────────────────────────────────────────────────────────
// High-Pass Filter
// Entfernt langsamen Drift (thermisch, ~10s Periode).
// alpha bestimmt die Grenzfrequenz:
//   alpha = 1 - exp(-2*pi * fc / fs)
//   fs = 100 Hz, fc = 0.05 Hz  →  alpha ≈ 0.00314
//   → Alles langsamer als ~20 Sekunden wird entfernt.
//   Für langsamere Grenzfrequenz: alpha verkleinern.
// ─────────────────────────────────────────────────────────────
#define HPF_ALPHA 0.00314f  // Grenzfrequenz ~0.05 Hz bei 100 Hz Abtastrate

class HighPassFilter {
public:
    HighPassFilter(float alpha) : _alpha(alpha), _lowpass(0.0f), _initialized(false) {}

    // Gibt nur die schnellen Anteile zurück (Drift wird entfernt).
    // Output ist zentriert um 0 → kein absoluter Winkel, sondern Abweichung.
    float update(float input) {
        if (!_initialized) {
            _lowpass     = input;
            _initialized = true;
            return 0.0f;
        }
        // Wrap-aware Delta für den Tiefpass berechnen
        float delta = input - _lowpass;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;

        _lowpass += _alpha * delta;
        _lowpass  = wrapAngle(_lowpass);

        // Hochpassanteil = aktuell minus langsamer Trend
        float hp = input - _lowpass;
        if (hp >  180.0f) hp -= 360.0f;
        if (hp < -180.0f) hp += 360.0f;
        return hp;
    }

    // Aktuellen Tiefpass-Trend abrufen (nützlich zum Debuggen)
    float getLowpass() const { return _lowpass; }

private:
    float _alpha;
    float _lowpass;
    bool  _initialized;

    static float wrapAngle(float a) {
        while (a <    0.0f) a += 360.0f;
        while (a >= 360.0f) a -= 360.0f;
        return a;
    }
};

// ─────────────────────────────────────────────────────────────
uint32_t      lastSample = 0;
float         offset     = 0.0f;
float         emaAngle   = 0.0f;
bool          emaInit    = false;
bool          calcMode   = false;
float         lastAngle  = 0.0f;
HighPassFilter hpf(HPF_ALPHA);  // Instanz des Filters

// ─────────────────────────────────────────────────────────────
float readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() == 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        raw &= 0x0FFF;
        return raw * 360.0f / 4096.0f;
    }
    return -1.0f;
}

// ─────────────────────────────────────────────────────────────
float wrapAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// ─────────────────────────────────────────────────────────────
float applyEMA(float newVal) {
    if (!emaInit) {
        emaAngle = newVal;
        emaInit  = true;
        return newVal;
    }
    float delta = newVal - emaAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    emaAngle = wrapAngle(emaAngle + EMA_ALPHA * delta);
    return emaAngle;
}

uint8_t readAGC() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AGC_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    esp_bt_controller_deinit();
    Serial.println("Wi-Fi and Bluetooth are now disabled.");

    Wire.begin();
    Wire.setClock(400000);

    delay(100);

    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    }
    offset = raw;
}

void printMagnitude() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x1B);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    uint16_t magnitude = (Wire.read() << 8) | Wire.read();
    magnitude &= 0x0FFF;
    Serial.printf(">magnitude:%d\n", magnitude);
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = micros();
    if (now - lastSample >= SAMPLE_INTERVAL_US) {
        lastSample = now;

        float raw = readRawAngle();
        if (raw < 0.0f) return;

        raw = wrapAngle(raw - offset);

        // ── CalcMode-Logik (unverändert) ──────────────────────
        float diff = raw - lastAngle;
        if (diff >  180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        if (abs(diff) >= 10) {
            calcMode = !calcMode;
            Serial.println("Switching Calc Mode");
        }
        lastAngle = raw;

        float angle = calcMode ? (360.0f - raw) : raw;

        // ── High-Pass Filter anwenden ─────────────────────────
        float hpFiltered = hpf.update(angle);
        // hpFiltered ≈ 0 bei reinem Drift
        // hpFiltered ≠ 0 bei echter schneller Bewegung

        // ── Ausgabe ───────────────────────────────────────────
        Serial.printf(">angle_raw:%.2f\n",          raw);
        Serial.printf(">angle_with_calc:%.2f\n",    angle);
        Serial.printf(">angle_hp_filtered:%.2f\n",  hpFiltered);
        Serial.printf(">angle_hp_trend:%.2f\n",     hpf.getLowpass()); // zum Debuggen: der Driftanteil

        printMagnitude();

        uint8_t agc = readAGC();
        Serial.print("AGC = ");
        Serial.println(agc);
    }
    delay(10);
}
*/

/*#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include "esp_bt.h"

#define AS5600_ADDR        0x36
#define REG_ANGLE_H        0x0E
#define SAMPLE_INTERVAL_US 10000  // 100 Hz
#define EMA_ALPHA          0.1f
#define AGC_REG     0x1A

// ─────────────────────────────────────────────────────────────
uint32_t lastSample = 0;
float    offset     = 0.0f;
float    emaAngle   = 0.0f;
bool     emaInit    = false;
bool     calcMode   = false;
float    lastAngle  = 0.0f;

// ─────────────────────────────────────────────────────────────
float readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_ANGLE_H);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() == 2) {
        uint16_t raw = (Wire.read() << 8) | Wire.read();
        raw &= 0x0FFF;
        return raw * 360.0f / 4096.0f;
    }
    return -1.0f;
}

// ─────────────────────────────────────────────────────────────
float wrapAngle(float a) {
    while (a <    0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// ─────────────────────────────────────────────────────────────
float applyEMA(float newVal) {
    if (!emaInit) {
        emaAngle = newVal;
        emaInit  = true;
        return newVal;
    }
    float delta = newVal - emaAngle;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    emaAngle = wrapAngle(emaAngle + EMA_ALPHA * delta);
    return emaAngle;
}

uint8_t readAGC() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AGC_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);


    // 1. Disconnect Wi-Fi and turn it OFF
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    // 2. Disable and de-initialize Bluetooth
    btStop();
    esp_bt_controller_deinit();
  
    Serial.println("Wi-Fi and Bluetooth are now disabled.");

    Wire.begin();
    Wire.setClock(400000);

    delay(100); // AS5600 startup settling

    float raw = readRawAngle();
    if (raw < 0.0f) {
        Serial.println("AS5600 not detected!");
    }
    offset = raw;
}

void printMagnitude() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x1B);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    uint16_t magnitude = (Wire.read() << 8) | Wire.read();
    magnitude &= 0x0FFF;
    Serial.printf(">magnitude:%d\n", magnitude);
}

// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = micros();
    if (now - lastSample >= SAMPLE_INTERVAL_US) {
        lastSample = now;

        float raw = readRawAngle();
        if (raw < 0.0f) return;

        raw = wrapAngle(raw - offset);
        // kleinste Winkeldifferenz berechnen (-180 .. +180)
        float diff = raw - lastAngle;
        if (diff > 180)
        {
             diff -= 360;
        }
        if (diff < -180)
        {
             diff += 360;
        }

        // Mindeständerung prüfen
        if (abs(diff) >= 10)
        {
            calcMode = !calcMode;
            Serial.println("Switching Calc Mode");
        }

        lastAngle = raw;
        
        float angle = raw;
        if(calcMode)
        {
            angle = 360 - raw;
        }

        //float filtered = applyEMA(angle);

        Serial.printf(">angle_raw:%.2f\n", raw);
        //Serial.printf(">angle_filtered:%.2f\n", filtered);
        Serial.printf(">angle_with_calc:%.2f\n", angle);

        printMagnitude();

        uint8_t agc = readAGC();

        Serial.print("AGC = ");
        Serial.println(agc);
    }
    delay(5);
}*/

/*

Auslesen des AGS Registers
#include <Wire.h>

#define AS5600_ADDR 0x36
#define AGC_REG     0x1A

void setup() {
  Serial.begin(115200);
  Wire.begin();

  uint8_t agc = readAGC();

  Serial.print("AGC = ");
  Serial.println(agc);
}

void loop() {
  uint8_t agc = readAGC();

  Serial.print("AGC = ");
  Serial.println(agc);

  delay(500);
}

uint8_t readAGC() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AGC_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDR, 1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}
*/