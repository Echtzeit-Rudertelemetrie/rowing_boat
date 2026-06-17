#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

// ── UART GPS  (Adafruit 746, MTK3339) ────────────────────────────────────────
// Wire: GPS-TX → IO16 (RX2),  GPS-RX → IO17 (TX2)
void gpsUartBegin(HardwareSerial &serial, int rx = 16, int tx = 17, uint32_t baud = 9600);
void gpsUartUpdate(HardwareSerial &serial, TinyGPSPlus &gps);

// ── I2C GPS  (u-blox NEO-M8T / ELT0332, default addr 0x42) ──────────────────
// Wire: IO21 (SDA), IO22 (SCL)
void gpsI2cBegin(uint8_t addr = 0x42);
void gpsI2cUpdate(TinyGPSPlus &gps);

// Mess-Rate per UBX-CFG-RATE setzen (1, 5 oder 10 Hz)
void gpsI2cSetRate(uint8_t hz);
