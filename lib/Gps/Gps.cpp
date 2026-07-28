#include "Gps.h"
#include <Wire.h>

bool Gps::probe() {
    Wire.beginTransmission(I2C_ADDR);
    return Wire.endTransmission() == 0;
}

// The FIFO level sits in the 16 bit register pair at 0xFD/0xFE, big endian.
// Reading it costs two bytes and saves fetching a full chunk of padding when
// the receiver has nothing queued.
uint16_t Gps::pending() {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(LEN_REGISTER);
    if (Wire.endTransmission(false) != 0) return 0xFFFF;   // no repeated start
    if (Wire.requestFrom((int)I2C_ADDR, 2) != 2) return 0xFFFF;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    return ((uint16_t)hi << 8) | lo;
}

void Gps::begin() {
    // A healthy idle I2C bus rests high. If either line is stuck low, stay off
    // the bus so update() never blocks on a wedged peripheral. Imu::begin()
    // calls Wire.begin() on its own, so the IMU keeps working either way.
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(20);
    if (digitalRead(SDA) != HIGH || digitalRead(SCL) != HIGH) return;

    Wire.begin();
    Wire.setClock(400000);     // matches Imu::begin(), whichever runs first
    wireReady_ = true;

    // Internal pull-ups make idle lines read high even with no module attached,
    // so probe the address before trusting the bus.
    busOk_     = probe();
    lastProbe_ = millis();
}

void Gps::update() {
    if (!wireReady_) return;
    const unsigned long now = millis();

    // The receiver may power up later than the hub, or be unplugged at runtime.
    // Retry occasionally instead of staying dead until the next reset.
    if (!busOk_) {
        if (now - lastProbe_ < PROBE_RETRY_MS) return;
        lastProbe_ = now;
        busOk_ = probe();
        if (!busOk_) return;
    }

    if (now - lastPoll_ < POLL_INTERVAL_MS) return;
    lastPoll_ = now;

    uint16_t waiting = pending();
    if (waiting == 0xFFFF) { busOk_ = false; return; }   // stopped answering
    if (waiting == 0) return;
    if (waiting > MAX_PER_POLL) waiting = MAX_PER_POLL;  // drain the rest next poll

    while (waiting > 0) {
        const uint8_t chunk = (waiting < READ_CHUNK) ? (uint8_t)waiting : READ_CHUNK;
        const uint8_t got   = Wire.requestFrom((int)I2C_ADDR, (int)chunk);
        if (got == 0) { busOk_ = false; return; }
        while (Wire.available()) {
            const char c = (char)Wire.read();
            if (c != (char)0xFF) gps_.encode(c);   // 0xFF is FIFO padding
        }
        waiting -= got;
    }
}

uint32_t Gps::locationAgeMs() const {
    return gps_.location.isValid() ? gps_.location.age() : UINT32_MAX;
}

GpsData Gps::data() {
    GpsData d{};
    d.valid = gps_.location.isValid();
    if (d.valid) {
        d.lat_e6 = (int32_t)(gps_.location.lat() * 1e6);
        d.lon_e6 = (int32_t)(gps_.location.lng() * 1e6);
    }
    d.speed_cms = gps_.speed.isValid()
        ? (uint16_t)constrain((long)(gps_.speed.mps() * 100.0 + 0.5), 0L, 65535L)
        : 0;
    d.course_cdeg = gps_.course.isValid()
        ? (uint16_t)constrain((long)(gps_.course.deg() * 100.0 + 0.5), 0L, 35999L)
        : 0;
    d.satellites = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    return d;
}
