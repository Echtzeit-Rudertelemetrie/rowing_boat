/***
 * BLE sender — telemetry hub (Seeed XIAO ESP32S3, env: prod_BLE_sender)
 *
 * Role in the boat:
 *   Receives oarlock data over UART (Serial1: D7=GPIO44 RX, D6=GPIO43 TX) from
 *   the ESP-NOW aggregator, reads the external GPS (u-blox, I2C/DDC @ 0x42 on
 *   D4/D5) and broadcasts everything via Bluetooth Low Energy (NimBLE) to up to
 *   BLE_MAX_CONN phones.
 *
 *   A central IMU is connected to the hub. Its values are SIMULATED for now
 *   (lib/SimData) and packed into the telemetry packet's angle region; a real
 *   IMU driver can replace SimData::imu() later without touching the wire format.
 *
 * Component code lives in lib/ (one folder per component, team convention):
 *   UartReceiver (lib/UartReceiver, [0xAA][0xBB] + MeasurementPack 132 B),
 *   Gps, Imu, SimData, BleSender (NimBLE).
 *
 * Packet id scheme (top 3 bits of espIdAndSeqenceNum, matches DataSender):
 *   id 0     -> telemetry: GpsData in force region, ImuData in angle region
 *   id 1..7  -> oarlock #id: force/angle as forwarded from UART
 */

#include <Arduino.h>
#include "UartReceiver.h"
#include "Gps.h"
#include "SimData.h"
#include "BleSender.h"

static UartReceiver receiver;
static Gps          gps;
static SimData      sim;
static BleSender    ble;

static uint32_t      telemSeq   = 0;
static unsigned long lastTelem  = 0;
static unsigned long lastStatus = 0;

static constexpr unsigned long TELEM_INTERVAL_MS  = 285;   // ~3.5 Hz GPS telemetry
static constexpr unsigned long STATUS_INTERVAL_MS = 1000;  // 1 Hz serial status

// Onboard LED as a serial-free liveness signal. XIAO ESP32S3 user LED = GPIO21,
// active LOW (LOW = lit).
static inline void ledOn()  { digitalWrite(LED_BUILTIN, LOW); }
static inline void ledOff() { digitalWrite(LED_BUILTIN, HIGH); }
static void blinkMarker(uint8_t times, unsigned ms) {
    for (uint8_t i = 0; i < times; i++) { ledOn(); delay(ms); ledOff(); delay(ms); }
}

// Dump the full contents of a packet being broadcast: id 0 is decoded as GPS +
// IMU telemetry, id 1..7 as the 32 force + 32 angle oarlock samples.
static void printPack(const char* tag, const MeasurementPack& p) {
    const uint8_t  id  = (p.espIdAndSeqenceNum >> 29) & 0x07u;
    const uint32_t seq =  p.espIdAndSeqenceNum & 0x1FFFFFFFu;

    if (id == 0) {
        GpsData g{};
        ImuData imu{};
        memcpy(&g,   p.force_values, sizeof(g));
        memcpy(&imu, p.angle_values, sizeof(imu));
        Serial.printf("%s id=0 seq=%lu TELEMETRY (%u B)\n",
                      tag, (unsigned long)seq, (unsigned)sizeof(p));
        Serial.printf("  GPS valid=%d sats=%u lat=%.6f lon=%.6f spd=%d course=%d\n",
                      g.valid, g.satellites, g.lat / 1e6, g.lon / 1e6,
                      (int)g.speed_mps, (int)g.course_deg);
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
    gps.begin();            // u-blox over I2C (D4/D5)

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
        printPack("RX->BLE", data);
    }

    unsigned long now = millis();

    // Notify: telemetry at ~3.5 Hz. id 0 marks telemetry; real GPS goes into the
    // force region, the (simulated) central IMU into the angle region.
    if (now - lastTelem >= TELEM_INTERVAL_MS) {
        lastTelem = now;
        MeasurementPack pkt{};
        pkt.espIdAndSeqenceNum = telemSeq++ & 0x1FFFFFFFu;   // id 0 (telemetry) | 29-bit seq

        GpsData g   = gps.data();
        ImuData imu = sim.imu();                              // central IMU, simulated for now
        static_assert(sizeof(g)   <= sizeof(pkt.force_values), "GpsData exceeds force region");
        static_assert(sizeof(imu) <= sizeof(pkt.angle_values), "ImuData exceeds angle region");
        memcpy(pkt.force_values, &g,   sizeof(g));
        memcpy(pkt.angle_values, &imu, sizeof(imu));

        ble.notifyMeasurement(pkt);
        printPack("TX", pkt);
    }

    // 1 Hz serial status + LED heartbeat (proves loop() is alive even with serial dead).
    if (now - lastStatus >= STATUS_INTERVAL_MS) {
        lastStatus = now;
        static bool hb = false;
        hb = !hb;
        digitalWrite(LED_BUILTIN, hb ? LOW : HIGH);
        GpsData g = gps.data();
        Serial.printf("GPS fix=%d sats=%u lat=%.6f lon=%.6f spd=%d | BLE conns=%u\n",
                      g.valid, g.satellites, g.lat / 1e6, g.lon / 1e6,
                      static_cast<int>(g.speed_mps), ble.connectionCount());
    }
}
