#include "Gps.h"
#include <Wire.h>

void Gps::begin() {
    // Guard against a dead/shorted bus: the nRF52 Wire layer's TWIM wait loops
    // have no timeout, so a stuck-low line would hang every transaction forever.
    // A healthy idle I2C bus rests high; if either line reads low, skip the bus.
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
    delayMicroseconds(20);
    busOk_ = (digitalRead(PIN_WIRE_SDA) == HIGH && digitalRead(PIN_WIRE_SCL) == HIGH);
    if (!busOk_) return;

    Wire.begin();              // primary bus: D4=SDA / D5=SCL
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
