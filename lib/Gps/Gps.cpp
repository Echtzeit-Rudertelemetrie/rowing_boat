#include "Gps.h"
#include <Wire.h>

void Gps::begin() {
    // Sanity-check the bus before using it: a healthy idle I2C bus rests high.
    // If either line reads low (stuck/shorted/unconnected module pulling low),
    // skip the bus so update() never blocks on a dead peripheral.
    // XIAO ESP32S3: SDA=GPIO5 (D4), SCL=GPIO6 (D5).
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(20);
    if (digitalRead(SDA) != HIGH || digitalRead(SCL) != HIGH) {
        busOk_ = false;        // lines stuck low -> never touch the bus
        return;
    }

    Wire.begin();              // primary bus: D4=SDA / D5=SCL

    // Probe the u-blox DDC address once. Internal pull-ups make idle lines read
    // high even with no module attached, so a line check alone can't tell the
    // module is missing. If it doesn't ACK, stay disabled so update() never
    // hammers a missing device (endless "i2cRead returned Error -1" spam).
    Wire.beginTransmission(I2C_ADDR);
    busOk_ = (Wire.endTransmission() == 0);
}

void Gps::update() {
    if (!busOk_) return;       // bus was dead at begin(); never touch it

    // u-blox DDC: read a chunk from the FIFO. Empty FIFO returns 0xFF padding,
    // which we skip; everything else is NMEA and goes into TinyGPSPlus.
    Wire.requestFrom((int)I2C_ADDR, (int)READ_CHUNK);
    while (Wire.available()) {
        char c = (char)Wire.read();
        if (c != (char)0xFF) {
            gps_.encode(c);
        }
    }
}

GpsData Gps::data() {
    GpsData d{};
    d.valid = gps_.location.isValid();
    if (d.valid) {
        d.lat = (int32_t)(gps_.location.lat() * 1e6);
        d.lon = (int32_t)(gps_.location.lng() * 1e6);
    }
    d.speed_mps  = gps_.speed.isValid()  ? (float)gps_.speed.mps()    : 0.0f;
    d.course_deg = gps_.course.isValid() ? (float)gps_.course.deg()   : 0.0f;
    d.satellites = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    return d;
}
