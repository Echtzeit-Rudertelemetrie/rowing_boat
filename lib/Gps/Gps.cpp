#include "gps.h"

// ── UART GPS ──────────────────────────────────────────────────────────────────
// void gpsUartBegin(HardwareSerial &serial, int rx, int tx, uint32_t baud) {
//     serial.begin(baud, SERIAL_8N1, rx, tx);
// }

// void gpsUartUpdate(HardwareSerial &serial, TinyGPSPlus &gps) {
//     while (serial.available()) {
//         gps.encode(serial.read());
//     }
// }

// ── I2C GPS (u-blox stream: 0xFD/0xFE = bytes available, 0xFF = data) ────────
static uint8_t _addr = 0x42;

// Sendet einen UBX-Befehl über I2C an den u-blox
static void ubxSend(const uint8_t *msg, size_t len) {
    Wire.beginTransmission(_addr);
    for (size_t i = 0; i < len; i++) Wire.write(msg[i]);
    Wire.endTransmission();
    delay(100);
}

void gpsI2cBegin(uint8_t addr) {
    _addr = addr;

    // UBX-CFG-RATE: Navigationsrate auf 10 Hz setzen (measRate = 100 ms)
    // Checksum über: 06 08 06 00 64 00 01 00 01 00 → CK_A=0x7A, CK_B=0x12
    static const uint8_t cfgRate[] = {
        0xB5, 0x62,              // UBX Header
        0x06, 0x08,              // CFG-RATE
        0x06, 0x00,              // Payload length
        0x64, 0x00,              // measRate = 100 ms  → 10 Hz
        0x01, 0x00,              // navRate  = 1
        0x01, 0x00,              // timeRef  = GPS
        0x7A, 0x12               // Checksum
    };
    ubxSend(cfgRate, sizeof(cfgRate));
}

void gpsI2cUpdate(TinyGPSPlus &gps) {
    // Read available byte count
    Wire.beginTransmission(_addr);
    Wire.write(0xFD);
    if (Wire.endTransmission(false) != 0) return;
    Wire.requestFrom(_addr, (uint8_t)2);
    if (Wire.available() < 2) return;
    uint16_t avail = ((uint16_t)Wire.read() << 8) | Wire.read();
    if (avail == 0 || avail == 0xFFFF) return;

    // Read stream in 32-byte chunks
    while (avail > 0) {
        uint8_t chunk = (uint8_t)min((uint16_t)32, avail);
        Wire.beginTransmission(_addr);
        Wire.write(0xFF);
        Wire.endTransmission(false);
        Wire.requestFrom(_addr, chunk);
        while (Wire.available()) {
            gps.encode((char)Wire.read());
            avail--;
        }
    }
}

void gpsI2cSetRate(uint8_t hz) {
    // UBX-CFG-RATE: Messrate setzen (1, 5 oder 10 Hz)
    uint16_t period_ms = 1000 / hz;

    // Payload: measRate, navRate=1, timeRef=1 (GPS)
    uint8_t payload[6] = {
        (uint8_t)(period_ms & 0xFF), (uint8_t)(period_ms >> 8),
        0x01, 0x00,   // navRate = 1
        0x01, 0x00    // timeRef = GPS
    };

    // Checksum über class(0x06) + id(0x08) + len(0x06,0x00) + payload
    uint8_t ck_a = 0, ck_b = 0;
    uint8_t header[] = { 0x06, 0x08, 0x06, 0x00 };
    for (uint8_t b : header)  { ck_a += b; ck_b += ck_a; }
    for (uint8_t b : payload) { ck_a += b; ck_b += ck_a; }

    Wire.beginTransmission(_addr);
    Wire.write(0xB5); Wire.write(0x62);           // UBX sync chars
    Wire.write(0x06); Wire.write(0x08);           // CFG-RATE
    Wire.write(0x06); Wire.write(0x00);           // length = 6
    for (uint8_t b : payload) Wire.write(b);
    Wire.write(ck_a); Wire.write(ck_b);
    Wire.endTransmission();
    delay(100);
}
