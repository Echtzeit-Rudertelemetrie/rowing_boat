#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
// #include <nrf52840.h>



// ── UART GPS  (Adafruit 746, MTK3339) ────────────────────────────────────────
// Wire: GPS-TX → IO16 (RX2),  GPS-RX → IO17 (TX2)
// void gpsUartBegin(HardwareSerial &serial, int rx = 16, int tx = 17, uint32_t baud = 9600);
// void gpsUartUpdate(HardwareSerial &serial, TinyGPSPlus &gps);

// Wire: D4 (SDA), D5 (SCL)
void gpsI2cBegin(uint8_t addr = 0x42);
void gpsI2cUpdate(TinyGPSPlus &gps);

// Mess-Rate per UBX-CFG-RATE setzen (1, 5 oder 10 Hz)
void gpsI2cSetRate(uint8_t hz);
