#pragma once

#include <Arduino.h>
#include "telemetry_packet.h"
#include "uart_rx.h"   // RowingPacket

// ─────────────────────────────────────────────────────────────
// BLE broadcast — platform-neutral API. The backend is chosen at compile
// time so the board can change without touching the rest of the firmware:
//   - nRF52 (current XIAO nRF52840): Adafruit Bluefruit (bundled with core)
//   - ESP32  (planned XIAO ESP32-S3): NimBLE-Arduino
// main only ever calls the functions below.
//
// One GATT service with two notify characteristics:
//   - TelemetryPacket (38 B)  — GPS + IMU, pushed at ~3.5 Hz
//   - RowingPacket   (133 B)  — oarlock data forwarded from UART, ~20 Hz
//
// 128-bit UUIDs (human-readable):
//   service     a1b2c3d4-0001-4a2b-9c3d-1234567890ab
//   telemetry   a1b2c3d4-0002-4a2b-9c3d-1234567890ab
//   rowing      a1b2c3d4-0003-4a2b-9c3d-1234567890ab
//
// Up to 4 phones may connect; ATT MTU is raised so the 133-byte RowingPacket
// fits in a single notification (needs MTU >= 136; we request the max 247).
// ─────────────────────────────────────────────────────────────

void    setupBLE();
void    bleNotifyTelemetry(const TelemetryPacket& pkt);
void    bleNotifyRowing(const RowingPacket& pkt);
uint8_t bleConnectionCount();
