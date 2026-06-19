/***
 * BLE sender — telemetry hub (Seeed XIAO nRF52840, env: prod_telemetry)
 *
 * Role in the boat:
 *   Collects oarlock data over UART (RX) from the ESP-NOW receiver, reads the
 *   external GPS (u-blox, I2C/DDC @ 0x42 on D4/D5) and the XIAO's onboard
 *   6-axis IMU, and broadcasts sequenced packets via Bluetooth Low Energy to
 *   up to 4 phones.
 *   Also responsible for battery charging management in the boat.
 *
 * Status (2026-06-18):
 *   Done  - GPS read (I2C, ~1 Hz debug) and IMU read (onboard LSM6DS3TR-C on
 *           Wire1, accel only, ~60 Hz) are refactored into lib/ble_sender
 *           (gps.* / imu.*); main streams both to Serial/teleplot (>imu/...).
 *   TODO  - UART input from the receiver, BLE broadcast, battery charging.
 */


#include <Arduino.h>

#include "gps.h"
#include "imu.h"
#include "telemetry_packet.h"
#include "uart_rx.h"
#include "ble_telemetry.h"

// GPS (u-blox, I2C/DDC) haengt extern am I2C-Bus: XIAO D4 -> SDA, D5 -> SCL.
// Die IMU ist auf dem XIAO-Board integriert (eigener interner I2C-Bus).
// Sensor-Logik liegt in lib/ble_sender.
const unsigned long GPS_DEBUG_INTERVAL_MS = 1000; // 1 Hz Status-Ausgabe

unsigned long lastGpsDebug = 0;
bool imuOk = false;

#ifdef PACKET_DEMO
// Synthetic TelemetryPacket stream for bench-testing the wire format without
// any I2C (the sensors hang on the damaged board). ~3.5 Hz, changing values.
static void demoEmitPacket() {
    static uint32_t seq = 0;

    TelemetryPacket pkt{};
    pkt.seq            = seq;
    pkt.gps.lat        = 48137154 + (int32_t)(seq % 200);  // ~48.137xxx deg * 1e6
    pkt.gps.lon        = 11575382 + (int32_t)(seq % 200);  // ~11.575xxx deg * 1e6
    pkt.gps.speed_mps  = 3.0f + (seq % 10) * 0.1f;
    pkt.gps.course_deg = (float)((seq * 5) % 360);
    pkt.gps.satellites = 7;
    pkt.gps.valid      = true;
    pkt.imu.acc_x      = ((int)(seq % 21) - 10) * 0.1f;        // -1.0 .. +1.0
    pkt.imu.acc_y      = ((int)((seq + 7) % 21) - 10) * 0.1f;
    pkt.imu.acc_z      = 9.81f;
    pkt.imu.timestamp_ms = millis();
    seq++;

    Serial.printf("pkt seq=%lu | gps lat=%ld lon=%ld spd=%.2f crs=%.1f sats=%u fix=%d "
                  "| imu ax=%.2f ay=%.2f az=%.2f t=%lu\n",
                  (unsigned long)pkt.seq, (long)pkt.gps.lat, (long)pkt.gps.lon,
                  pkt.gps.speed_mps, pkt.gps.course_deg, pkt.gps.satellites, pkt.gps.valid,
                  pkt.imu.acc_x, pkt.imu.acc_y, pkt.imu.acc_z, (unsigned long)pkt.imu.timestamp_ms);

    const uint8_t* raw = (const uint8_t*)&pkt;
    Serial.printf("raw[%u]: ", (unsigned)sizeof(pkt));
    for (size_t i = 0; i < sizeof(pkt); i++) {
        Serial.printf("%02X ", raw[i]);
    }
    Serial.println();

    // Push over BLE so a phone / nRF Connect sees live notifications
    bleNotifyTelemetry(pkt);

    RowingPacket rp{};
    rp.board_id = 1;
    rp.seq      = pkt.seq;
    for (int i = 0; i < UART_RX_PACKET_VALUES; i++) {
        rp.force[i] = (uint16_t)(1000 + ((pkt.seq + i) % 500));
        rp.angle[i] = (uint16_t)(((pkt.seq * 3) + i) % 360);
    }
    bleNotifyRowing(rp);

    delay(285); // ~3.5 Hz, the planned TelemetryPacket notification rate
}
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    delay(300); // give a freshly-attached monitor time to catch the boot log

#ifdef PACKET_DEMO
    Serial.println("\n=== TelemetryPacket DEMO (synthetic, no I2C) ===");
    Serial.printf("sizeof: TelemetryPacket=%u GpsData=%u ImuData=%u\n",
                  (unsigned)sizeof(TelemetryPacket), (unsigned)sizeof(GpsData),
                  (unsigned)sizeof(ImuData));
    setupBLE();
    Serial.println("BLE up - advertising as 'RowingBoat' (connect with nRF Connect)");
    Serial.flush();
    return;
#endif

    Serial.println("\n=== Telemetry: GPS (I2C) + IMU ===");
    Serial.flush();

    Serial.println("[boot] setupGPS...");
    Serial.flush();
    setupGPS();
    Serial.println("[boot] setupGPS done");
    Serial.flush();

#ifndef SKIP_IMU
    Serial.println("[boot] setupIMU...");
    Serial.flush();
    imuOk = setupIMU();
    Serial.printf("[boot] setupIMU done, imuOk=%d\n", imuOk);
    Serial.flush();

    if (!imuOk) {
        Serial.println("IMU (LSM6DS3) not found - continuing without IMU");
    } else {
        Serial.println("IMU (LSM6DS3) OK");
    }
    Serial.flush();
#else
    Serial.println("[boot] setupIMU SKIPPED (compiled with -DSKIP_IMU)");
    Serial.flush();
    imuOk = false;
#endif

    Serial.println("[boot] setupUartRx...");
    Serial.flush();
    setupUartRx();
    Serial.println("[boot] setupUartRx done");
    Serial.flush();

    Serial.println("[boot] setupBLE...");
    Serial.flush();
    setupBLE();
    Serial.println("[boot] setupBLE done - advertising as 'RowingBoat'");
    Serial.flush();
}

void loop() {
#ifdef PACKET_DEMO
    demoEmitPacket();
    return;
#endif
    updateGPS();
    updateUartRx();

    unsigned long now = millis();

    // Keep the latest accelerometer sample for the telemetry packet
    static ImuData lastImu{};
    if (imuOk) {
        ImuData s;
        if (sampleIMU(s)) {
            lastImu = s;
            Serial.printf(">imu/Acc_x: %.4f\n", s.acc_x);
            Serial.printf(">imu/Acc_y: %.4f\n", s.acc_y);
            Serial.printf(">imu/Acc_z: %.4f\n", s.acc_z);
        }
    }

    // BLE notify A: TelemetryPacket (GPS + IMU) at ~3.5 Hz
    static unsigned long lastTelem = 0;
    static uint32_t      telemSeq  = 0;
    if (now - lastTelem >= 285) {
        lastTelem = now;
        TelemetryPacket pkt{};
        pkt.seq = telemSeq++;
        pkt.gps = getGpsData();
        pkt.imu = lastImu;
        bleNotifyTelemetry(pkt);
    }

    // BLE notify B: forward the latest RowingPacket per board at ~20 Hz
    static unsigned long lastRow = 0;
    if (now - lastRow >= 50) {
        lastRow = now;
        RowingPacket rp;
        for (uint8_t id = 0; id < UART_RX_MAX_BOARDS; id++) {
            if (uartRxGetLatest(id, rp)) bleNotifyRowing(rp);
        }
    }

    // 1 Hz status to Serial
    if (now - lastGpsDebug >= GPS_DEBUG_INTERVAL_MS) {
        lastGpsDebug = now;
        GpsData g = getGpsData();
        Serial.printf("GPS: lat=%.6f lon=%.6f speed=%.2f m/s heading=%.1f deg sats=%u fix=%d\n",
                      g.lat / 1e6, g.lon / 1e6, g.speed_mps, g.course_deg,
                      g.satellites, g.valid);
        Serial.printf("BLE connections: %u | UART frames good=%lu crcErr=%lu\n",
                      bleConnectionCount(),
                      (unsigned long)uartRxGoodFrames(), (unsigned long)uartRxCrcErrors());
        RowingPacket rp;
        for (uint8_t id = 0; id < UART_RX_MAX_BOARDS; id++) {
            if (uartRxGetLatest(id, rp)) {
                Serial.printf("UART board %u: seq=%lu force[0]=%u angle[0]=%u\n",
                              id, (unsigned long)rp.seq, rp.force[0], rp.angle[0]);
            }
        }
    }
}
