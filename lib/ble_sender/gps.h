#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// GPS
// Stub for the dedicated GPS ESP32 (env: prod_gps, board nodemcu-32s).
// TODO: pick GPS module (e.g. u-blox NEO-6M/M8N) and implement
//       NMEA parsing (e.g. via TinyGPS++ on Serial2).
// ─────────────────────────────────────────────────────────────


struct __attribute__((packed)) GpsData {
    int32_t  lat;           // millionths of a degree (gps.location.lat() * 1e6)
    int32_t  lon;           // millionths of a degree
    float    speed_mps;     //meter per second
    float    course_deg;
    uint8_t  satellites;
    bool     valid;
};                          // = 4+4+4+4+1+1 = 18 bytes


// Initialize the GPS module (UART pins, baud rate)
void setupGPS();

// Poll the GPS module and publish position/speed data
void updateGPS();

// Snapshot the most recent parsed values into a GpsData struct.
GpsData getGpsData();
