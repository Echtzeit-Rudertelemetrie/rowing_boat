/***
 * BLE sender — SIMULATION build (env: sim_BLE_sender)
 *
 * Broadcasts synthetic data over BLE with rolling sequence numbers, NO sensors:
 *   - telemetry (id 0): GPS + IMU
 *   - oarlock #1 (id 1) and oarlock #2 (id 2): force/angle
 * Lets you exercise the BLE / receiver / phone pipeline without GPS/IMU/UART
 * hardware. Each packet is also dumped as hex on Serial so you can diff the wire
 * bytes against a decoder.
 */

#include <Arduino.h>
#include "BleSender.h"
#include "SimData.h"

static BleSender ble;
static SimData   sim;

static unsigned long lastCycle = 0;
static constexpr unsigned long CYCLE_MS = 500;   // one full sim cycle every 0.5 s

static void dumpHex(const char* tag, const MeasurementPack& p) {
    const uint8_t* b = (const uint8_t*)&p;
    const uint8_t  id  = (p.idAndSeq >> 28) & 0x0F;
    const uint32_t seq =  p.idAndSeq & 0x0FFFFFFFu;
    Serial.printf("%s id=%u seq=%lu: ", tag, id, (unsigned long)seq);
    for (size_t i = 0; i < sizeof(p); i++) Serial.printf("%02X", b[i]);
    Serial.println();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println("\n=== BLE telemetry hub - SIMULATION ===");
    if (ble.begin()) Serial.println("BLE up - advertising as 'RowingBoat' (SIM)");
    else             Serial.println("!!! BLE begin() FAILED");
}

void loop() {
    const unsigned long now = millis();
    if (now - lastCycle < CYCLE_MS) return;
    lastCycle = now;

    static bool hb = false; hb = !hb;
    digitalWrite(LED_BUILTIN, hb ? LED_STATE_ON : !LED_STATE_ON);

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
