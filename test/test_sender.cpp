#include <Arduino.h>
#include "espnow.h"

#define BOARD_ID        1
#define ESPNOW_CHANNEL  11

constexpr float FORCE_MIN_N     =    0.0f;
constexpr float FORCE_MAX_N     = 1000.0f;
constexpr float ANGLE_MIN_DEG   =  -90.0f;
constexpr float ANGLE_MAX_DEG   =  +90.0f;

constexpr uint32_t STROKE_PERIOD_MS = 2000;
constexpr float    DRIVE_FRACTION   = 0.40f;
constexpr float    ANGLE_CATCH_DEG  = -60.0f;
constexpr float    ANGLE_FINISH_DEG =  40.0f;
constexpr float    FORCE_PEAK_N     = 800.0f;
constexpr uint16_t SAMPLE_PERIOD_MS = 5;

#if BOARD_ID == 1
    constexpr float PHASE_OFFSET = 0.000f;
    constexpr float FORCE_SCALE  = 1.00f;
    constexpr const char* SIDE   = "LINKS";
#elif BOARD_ID == 2
    constexpr float PHASE_OFFSET = 0.008f;
    constexpr float FORCE_SCALE  = 0.95f;
    constexpr const char* SIDE   = "RECHTS";
#else
    #error "BOARD_ID muss 1 oder 2 sein"
#endif

static uint8_t  hub_mac[6]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint16_t forceBuffer[PACKET_VALUES];
static uint16_t angleBuffer[PACKET_VALUES];
static uint8_t  bufferIndex  = 0;
static uint32_t packetSeq    = 0;

uint16_t quantize(float v, float vmin, float vmax, uint8_t bits) {
    const uint32_t maxInt = (1UL << bits) - 1;
    if (v <= vmin) return 0;
    if (v >= vmax) return (uint16_t)maxInt;
    return (uint16_t)(((v - vmin) / (vmax - vmin)) * maxInt + 0.5f);
}

void simulateStroke(float& angle_deg, float& force_N) {
    float cycle = fmodf((millis() / (float)STROKE_PERIOD_MS) + PHASE_OFFSET, 1.0f);
    if (cycle < DRIVE_FRACTION) {
        float p = cycle / DRIVE_FRACTION;
        angle_deg = ANGLE_CATCH_DEG + (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG) * p;
        float shape;
        if (p < 0.4f) {
            shape = sinf(p / 0.4f * PI * 0.5f);
        } else {
            shape = cosf((p - 0.4f) / 0.6f * PI * 0.5f);
        }
        force_N = FORCE_PEAK_N * FORCE_SCALE * shape;
    } else {
        float p = (cycle - DRIVE_FRACTION) / (1.0f - DRIVE_FRACTION);
        angle_deg = ANGLE_FINISH_DEG - (ANGLE_FINISH_DEG - ANGLE_CATCH_DEG) * p;
        force_N = 0.0f;
    }
    force_N   += (random(-100, 101) / 100.0f) * 4.0f;
    angle_deg += (random(-100, 101) / 100.0f) * 0.2f;
    if (force_N < 0.0f) force_N = 0.0f;
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.printf("\n=== Dolle-ESP #%d (%s) ===\n", BOARD_ID, SIDE);
    Serial.printf("Sende %d Werte pro Paket, %d-fache Wiederholung\n", PACKET_VALUES, PACKET_RETRIES);

    uint8_t local_mac[6];
    WiFi.macAddress(local_mac);
    Serial.printf("Meine MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  local_mac[0], local_mac[1], local_mac[2],
                  local_mac[3], local_mac[4], local_mac[5]);

    espnow_init_sender(BOARD_ID, hub_mac);

    randomSeed((uint32_t)BOARD_ID * 12345u + micros());
    Serial.println("Bereit - sammle und sende gebuendelt\n");
}

void loop() {
    static uint32_t lastSample = 0;
    uint32_t now = millis();

    if (now - lastSample < SAMPLE_PERIOD_MS) return;
    lastSample = now;

    float angle_deg, force_N;
    simulateStroke(angle_deg, force_N);

    forceBuffer[bufferIndex] = quantize(force_N,   FORCE_MIN_N,   FORCE_MAX_N,   11);
    angleBuffer[bufferIndex] = quantize(angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG, 10);
    bufferIndex++;

    if (bufferIndex == PACKET_VALUES) {
        RowingPacket pkt;
        pkt.board_id = BOARD_ID;
        pkt.seq = packetSeq++;
        memcpy(pkt.force, forceBuffer, sizeof(forceBuffer));
        memcpy(pkt.angle, angleBuffer, sizeof(angleBuffer));

        for (int retry = 0; retry < PACKET_RETRIES; retry++) {
            esp_now_send(hub_mac, (const uint8_t*)&pkt, sizeof(pkt));
            if (retry < PACKET_RETRIES - 1) delay(1);
        }

        static uint32_t lastLog = 0;
        if (now - lastLog > 500) {
            lastLog = now;
            Serial.printf("Seq=%u (%dfach) | Kraft=%u | Winkel=%u\n",
                          pkt.seq, PACKET_RETRIES, pkt.force[0], pkt.angle[0]);
        }

        bufferIndex = 0;
    }
}