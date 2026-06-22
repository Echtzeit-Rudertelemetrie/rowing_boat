/***
 * BLE sender — telemetry hub (Seeed XIAO nRF52840 Sense, env: prod_BLE_sender)
 *
 * Role in the boat:
 *   Receives oarlock data over UART (Serial1, D7=RX) from the ESP-NOW
 *   aggregator, reads the external GPS (u-blox, I2C/DDC @ 0x42 on D4/D5) and the
 *   onboard 6-axis IMU (LSM6DS3TR-C, accel only), and broadcasts everything via
 *   Bluetooth Low Energy to up to 4 phones. (No battery telemetry — the pack
 *   only charges, it never reports state.)
 *
 * Component code lives in lib/ (one folder per component, team convention):
 *   UartReceiver (lib/UartReceiver, [0xAA][0xBB] + MeasurementPack 84 B),
 *   Gps, Imu, BleSender.
 *
 * BLE: one service, two notify characteristics —
 *   TelemetryPacket (38 B, GPS+IMU) ~3.5 Hz,
 *   MeasurementPack (84 B, forwarded oarlock data) as it arrives.
 */

#include <Arduino.h>
#include "UartReceiver.h"
#include "Gps.h"
#include "Imu.h"
#include "BleSender.h"

static UartReceiver receiver;
static Gps          gps;
static Imu          imu;
static BleSender    ble;

static bool          imuOk      = false;
static ImuData       lastImu{};
static uint32_t      telemSeq   = 0;
static unsigned long lastTelem  = 0;
static unsigned long lastStatus = 0;

static constexpr unsigned long TELEM_INTERVAL_MS  = 285;   // ~3.5 Hz telemetry
static constexpr unsigned long STATUS_INTERVAL_MS = 1000;  // 1 Hz serial status

// Onboard LED as a serial-free liveness signal (USB-CDC is unreliable on this
// board). Polarity-robust via the variant's LED_STATE_ON.
static inline void ledOff()    { digitalWrite(LED_BUILTIN, !LED_STATE_ON); }
static inline void ledOn()     { digitalWrite(LED_BUILTIN,  LED_STATE_ON); }
static void blinkMarker(uint8_t times, unsigned ms) {
    for (uint8_t i = 0; i < times; i++) { ledOn(); delay(ms); ledOff(); delay(ms); }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    ledOff();
    blinkMarker(1, 600);    // CP1: setup() entered (one LONG blink). No CP1 => the
                            // firmware isn't running (bad flash / fault / wrong LED).

    Serial.begin(115200);   // USB-CDC; output appears only if a host is attached

    Serial.println("\n=== BLE telemetry hub ===");

    // BLE FIRST: bring up advertising before touching any I2C sensor. The nRF52
    // Wire layer's TWIM transactions have no timeout, so a dead/shorted sensor
    // bus could otherwise hang setup() and the hub would never become
    // discoverable. The sensor drivers below also guard against a stuck bus, but
    // starting BLE first guarantees the hub is always visible.
    if (ble.begin()) {
        Serial.println("BLE up - advertising as 'RowingBoat'");
        blinkMarker(3, 120);    // 3 slow blinks: advertising is up
    } else {
        // SoftDevice refused the config (almost always NO_MEM: too many
        // connections / MTU too large). Distinct rapid burst so it's obvious on
        // the bench that the radio is NOT on air — no more silent dead BLE.
        Serial.println("!!! BLE begin() FAILED - SoftDevice NO_MEM? Lower BLE_ATT_MTU or BLE_MAX_CONN");
        blinkMarker(10, 60);
    }

    receiver.begin();

    gps.begin();

    imuOk = imu.begin();
    Serial.printf("IMU (LSM6DS3TR-C): %s\n", imuOk ? "OK" : "not found");

    Serial.println("setup done");
}

void loop() {
    gps.update();
    receiver.update();

    // Forward oarlock data over BLE the moment a full UART frame arrives.
    if (receiver.isNewDataAvailable()) {
        MeasurementPack data = receiver.getLatestPacket();
        ble.notifyMeasurement(data);
    }

    // Keep the latest accelerometer sample for the telemetry packet.
    if (imuOk) {
        ImuData s;
        if (imu.sample(s)) lastImu = s;
    }

    unsigned long now = millis();

    // Notify A: TelemetryPacket (GPS + IMU) at ~3.5 Hz.
    if (now - lastTelem >= TELEM_INTERVAL_MS) {
        lastTelem = now;
        TelemetryPacket pkt{};
        pkt.seq = telemSeq++;
        pkt.gps = gps.data();
        pkt.imu = lastImu;
        ble.notifyTelemetry(pkt);
    }

    // 1 Hz status to the serial monitor + LED heartbeat (proves loop() is alive
    // even with serial dead): a slow blink => setup() finished and loop runs.
    if (now - lastStatus >= STATUS_INTERVAL_MS) {
        lastStatus = now;
        static bool hb = false;
        hb = !hb;
        digitalWrite(LED_BUILTIN, hb ? LED_STATE_ON : !LED_STATE_ON);
        GpsData g = gps.data();
        Serial.printf("GPS fix=%d sats=%u lat=%.6f lon=%.6f spd=%.2f | "
                      "IMU=%s | BLE conns=%u\n",
                      g.valid, g.satellites, g.lat / 1e6, g.lon / 1e6, g.speed_mps,
                      imuOk ? "ok" : "--", ble.connectionCount());
    }
}
