#pragma once
#include <Arduino.h>

#define ESPNOW_MAX_BOARD_ID 255

// Konfiguration ESP-NOW
static constexpr uint8_t ESPNOW_CHANNEL = 11;
// 32 Werte pro Paket -> MeasurementPack = 4 + 32*2 + 32*2 = 132 Bytes.
// Muss mit dem Sender (Branch sensor_miniesp_code / DataSender) uebereinstimmen,
// sonst verwirft der Empfaenger wegen Laengenpruefung (len != sizeof) alles.
// Achtung: bei Aenderung auch BLE_ATT_MTU in lib/BleSender/BleSender.h anpassen
// (MeasurementPack muss in eine BLE-Notification passen).
static constexpr uint8_t PACKET_VALUES = 32;

// Konfiguration UART
static constexpr int UART2_RX_PIN = 16;
static constexpr int UART2_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;

enum class EventType : uint8_t {
  ReadSensor1,
  ReadSensor2,
  SendData
};

typedef struct __attribute__((packed)) {
  uint32_t espIdAndSeqenceNum;
  u_int16_t force_values[PACKET_VALUES];
  u_int16_t angle_values[PACKET_VALUES];
} MeasurementPack;

// ── Kodierung von espIdAndSeqenceNum ────────────────────────────────────────
// 4 Bit ID (oben) | 28 Bit Sequenznummer — vereinheitlichtes Hub-Design
// (Entscheidung 2026-06-24, umgesetzt 2026-07-22; vorher 3/29 Bit).
//   id 0      -> Telemetrie: force_values traegt GpsData, angle_values ImuData
//   id 1..15  -> Dolle #id: force/angle wie benannt
// Immer ueber diese Helfer kodieren/dekodieren, nie mit eigenen Shifts: die
// Kodierung ist schon einmal zwischen Sender, Empfaenger und Phone-App
// auseinandergelaufen. Die Phone-App muss dasselbe Schema lesen.
static constexpr uint8_t  IDSEQ_ID_SHIFT = 28;
static constexpr uint8_t  IDSEQ_ID_MASK  = 0x0Fu;
static constexpr uint32_t IDSEQ_SEQ_MASK = 0x0FFFFFFFu;

static constexpr uint32_t packIdSeq(uint8_t id, uint32_t seq) {
  return (static_cast<uint32_t>(id & IDSEQ_ID_MASK) << IDSEQ_ID_SHIFT) |
         (seq & IDSEQ_SEQ_MASK);
}

static constexpr uint8_t idFromIdSeq(uint32_t idAndSeq) {
  return static_cast<uint8_t>((idAndSeq >> IDSEQ_ID_SHIFT) & IDSEQ_ID_MASK);
}

static constexpr uint32_t seqFromIdSeq(uint32_t idAndSeq) {
  return idAndSeq & IDSEQ_SEQ_MASK;
}

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
};