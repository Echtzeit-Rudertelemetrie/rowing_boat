#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// GPS
// Stub for the dedicated GPS ESP32 (env: prod_gps, board nodemcu-32s).
// TODO: pick GPS module (e.g. u-blox NEO-6M/M8N) and implement
//       NMEA parsing (e.g. via TinyGPS++ on Serial2).
// ─────────────────────────────────────────────────────────────

// Initialize the GPS module (UART pins, baud rate)
void setupGPS();

// Poll the GPS module and publish position/speed data
void updateGPS();
