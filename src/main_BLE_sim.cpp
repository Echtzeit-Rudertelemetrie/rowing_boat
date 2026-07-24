/***
 * BLE sender — SIMULATION build (Seeed XIAO ESP32S3, env: sim_BLE_sender)
 *
 * Broadcasts synthetic data over BLE with rolling sequence numbers, NO sensors:
 *   - telemetry (id 0): simulated GPS + IMU
 *   - oarlock #1 (id 1) and oarlock #2 (id 2): force/angle
 * Lets you exercise the BLE / phone pipeline without GPS/IMU/UART hardware. Each
 * packet is also dumped as hex on Serial so you can diff the wire bytes against
 * the phone-app decoder.
 */

#include <Arduino.h>
#include "BleSender.h"
#include "SimData.h"

static BleSender ble;
static SimData   sim;

static unsigned long lastCycle = 0;
static constexpr unsigned long CYCLE_MS = 500;   // one full sim cycle every 0.5 s

// XIAO ESP32S3 user LED = GPIO21, active LOW.
static inline void ledSet(bool on) { digitalWrite(LED_BUILTIN, on ? LOW : HIGH); }

static void dumpHex(const char* tag, const MeasurementPack& p) {
    const uint8_t* b = (const uint8_t*)&p;
    const uint8_t  id  = idFromIdSeq(p.espIdAndSeqenceNum);
    const uint32_t seq = seqFromIdSeq(p.espIdAndSeqenceNum);
    Serial.printf("%s id=%u seq=%lu: ", tag, id, (unsigned long)seq);
    for (size_t i = 0; i < sizeof(p); i++) Serial.printf("%02X", b[i]);
    Serial.println();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    ledSet(false);
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) { /* wait for USB, max 3s */ }

    Serial.println("\n=== BLE telemetry hub - SIMULATION (XIAO ESP32S3) ===");
    if (ble.begin()) Serial.println("BLE up - advertising as 'RowingBoat' (SIM)");
    else             Serial.println("!!! BLE begin() FAILED");
}

void loop() {
    const unsigned long now = millis();
    if (now - lastCycle < CYCLE_MS) return;
    lastCycle = now;

    static bool hb = false; hb = !hb;
    ledSet(hb);

    const MeasurementPack tele = sim.telemetry();   // id 0
    const MeasurementPack d1   = sim.dolle(1);      // id 1
    const MeasurementPack d2   = sim.dolle(2);      // id 2

    ble.notifyMeasurement(tele);
    ble.notifyMeasurement(d1);
    ble.notifyMeasurement(d2);

    dumpHex("TELE", tele);
    dumpHex("DOL1", d1);
    dumpHex("DOL2", d2);
    Serial.printf("conns=%u\n", ble.connectionCount());
}
