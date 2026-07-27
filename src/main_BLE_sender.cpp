/***
 * BLE sender — telemetry hub (Seeed XIAO ESP32S3, env: prod_BLE_sender)
 *
 * Role in the boat:
 *   Receives oarlock data over UART (Serial1: D7=GPIO44 RX, D6=GPIO43 TX) from
 *   the ESP-NOW aggregator, reads the NEO-M8T over Serial2
 *   (D10=GPIO9 RX, D9=GPIO8 TX) and broadcasts everything via BLE to up to
 *   BLE_MAX_CONN phones.
 *
 *   A central ICM-20948 is connected to the hub over I2C. Its acceleration is
 *   packed into the telemetry packet's angle region.
 *
 * Component code lives in lib/ (one folder per component, team convention):
 *   UartReceiver (lib/UartReceiver, [0xAA][0xBB] + MeasurementPack 36 B),
 *   Gps, Imu, BleSender (NimBLE).
 *
 * Packet id scheme (top 4 bits of espIdAndSeqenceNum):
 *   id 0     -> telemetry: GpsData in force region, ImuData in angle region
 *   id 1..15 -> oarlock #id: force/angle as forwarded from UART
 */

#include <Arduino.h>
#include "UartReceiver.h"
#include "UartGps.h"
#include "Imu.h"
#include "BleSender.h"

#ifndef BLE_SERIAL_DUMP
#define BLE_SERIAL_DUMP 0
#endif

static UartReceiver receiver;
static UartGps      gps;
static Imu          imu;
static BleSender    ble;

static uint32_t      telemSeq   = 0;
static unsigned long lastTelem  = 0;
static unsigned long lastStatus = 0;

static constexpr unsigned long TELEM_INTERVAL_MS  = 20;    // 50 Hz IMU telemetry
static constexpr unsigned long STATUS_INTERVAL_MS = 1000;  // 1 Hz serial status

// Onboard LED as a serial-free liveness signal. XIAO ESP32S3 user LED = GPIO21,
// active LOW (LOW = lit).
static inline void ledOn()  { digitalWrite(LED_BUILTIN, LOW); }
static inline void ledOff() { digitalWrite(LED_BUILTIN, HIGH); }
static void blinkMarker(uint8_t times, unsigned ms) {
    for (uint8_t i = 0; i < times; i++) { ledOn(); delay(ms); ledOff(); delay(ms); }
}

// Dump the full contents of a packet being broadcast: id 0 is decoded as GPS +
// IMU telemetry, id 1..15 as the 8 force + 8 angle oarlock samples.
static void printPack(const char* tag, const MeasurementPack& p) {
    const uint8_t  id  = idFromIdSeq(p.espIdAndSeqenceNum);
    const uint32_t seq = seqFromIdSeq(p.espIdAndSeqenceNum);

    if (id == 0) {
        GpsData g{};
        ImuData imu{};
        memcpy(&g,   p.force_values, sizeof(g));
        memcpy(&imu, p.angle_values, sizeof(imu));
        Serial.printf("%s id=0 seq=%lu TELEMETRY (%u B)\n",
                      tag, (unsigned long)seq, (unsigned)sizeof(p));
        Serial.printf("  GPS valid=%d sats=%u lat=%.6f lon=%.6f spd=%d course=%d\n",
                      g.valid, g.satellites, g.lat_e6 / 1e6, g.lon_e6 / 1e6,
                      (int)g.speed_cms, (int)g.course_cdeg);
        Serial.printf("  IMU acc=[%.3f %.3f %.3f] t=%lu\n",
                      imu.acc_x, imu.acc_y, imu.acc_z, (unsigned long)imu.timestamp_ms);
        return;
    }

    Serial.printf("%s id=%u seq=%lu OARLOCK (%u B)\n",
                  tag, id, (unsigned long)seq, (unsigned)sizeof(p));
    Serial.print("  force:");
    for (uint8_t i = 0; i < PACKET_VALUES; i++) Serial.printf(" %u", p.force_values[i]);
    Serial.print("\n  angle:");
    for (uint8_t i = 0; i < PACKET_VALUES; i++) Serial.printf(" %u", p.angle_values[i]);
    Serial.println();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    ledOff();
    blinkMarker(1, 600);    // CP1: setup() entered (one LONG blink).

    Serial.begin(115200);   // native USB-CDC; output appears only if a host is attached
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) { /* wait for USB, max 3s */ }

    Serial.println("\n=== BLE telemetry hub (XIAO ESP32S3) ===");

    // BLE first so the hub is discoverable regardless of sensor state.
    if (ble.begin()) {
        Serial.println("BLE up - advertising as 'RowingBoat'");
        blinkMarker(3, 120);
    } else {
        Serial.println("!!! BLE begin() FAILED");
        blinkMarker(10, 60);
    }

    receiver.begin();       // Serial1 UART from the ESP-NOW aggregator
    gps.begin();            // NEO-M8T: Serial2, D10 RX / D9 TX

    if (imu.begin()) Serial.println("IMU up - ICM-20948 streaming real acceleration");
    else             Serial.println("!!! IMU begin() FAILED - check I2C wiring/address");

    Serial.println("setup done");
}

void loop() {
    gps.update();
    receiver.update();

    // Forward oarlock (and any upstream IMU) data over BLE the moment a full
    // UART frame arrives.
    if (receiver.isNewDataAvailable()) {
        MeasurementPack data = receiver.getLatestPacket();
        ble.notifyMeasurement(data);
#if BLE_SERIAL_DUMP
        printPack("RX->BLE", data);
#endif
    }

    unsigned long now = millis();

    // Notify at 50 Hz. id 0 marks telemetry; the latest GPS fix goes into the
    // force region and the real central IMU sample into the angle region.
    if (now - lastTelem >= TELEM_INTERVAL_MS) {
        lastTelem = now;
        MeasurementPack pkt{};
        pkt.espIdAndSeqenceNum = packIdSeq(0, telemSeq++);

        GpsData g   = gps.data();
        ImuData imuSample = imu.read();
        static_assert(sizeof(g)   <= sizeof(pkt.force_values), "GpsData exceeds force region");
        static_assert(sizeof(imuSample) <= sizeof(pkt.angle_values), "ImuData exceeds angle region");
        memcpy(pkt.force_values, &g,   sizeof(g));
        memcpy(pkt.angle_values, &imuSample, sizeof(imuSample));

        ble.notifyMeasurement(pkt);
#if BLE_SERIAL_DUMP
        printPack("TX", pkt);
#endif
    }

    // 1 Hz serial status + LED heartbeat (proves loop() is alive even with serial dead).
    if (now - lastStatus >= STATUS_INTERVAL_MS) {
        lastStatus = now;
        static bool hb = false;
        hb = !hb;
        digitalWrite(LED_BUILTIN, hb ? LOW : HIGH);
        GpsData g = gps.data();
        const uint32_t age = gps.locationAgeMs();
        Serial.printf("GPS fix=%d sats=%u lat=%.6f lon=%.6f spd=%.2f m/s"
                      " | uart_chars=%lu nmea_ok=%lu nmea_bad=%lu age_ms=",
                      g.valid, g.satellites, g.lat_e6 / 1e6, g.lon_e6 / 1e6,
                      g.speed_cms / 100.0,
                      static_cast<unsigned long>(gps.charsProcessed()),
                      static_cast<unsigned long>(gps.passedChecksums()),
                      static_cast<unsigned long>(gps.failedChecksums()));
        if (age == UINT32_MAX) Serial.print("n/a");
        else Serial.print(age);
        Serial.printf(" | IMU=%s | BLE conns=%u\n",
                      imu.available() ? "ok" : "missing", ble.connectionCount());
    }
}
