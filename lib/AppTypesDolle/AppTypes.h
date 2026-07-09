#pragma once
#include <Arduino.h>

// TODO/ABSTIMMUNG: Team-main definiert PACKET_VALUES = 20 -> MeasurementPack =
// 84 Bytes. Dieser Branch sendet mit 32 Werten 132 Bytes — der Empfaenger prueft
// die Paketlaenge (len != sizeof) und verwirft alles, es kommt also NICHTS am
// Hub an. Vorschlag: vor Feldtests auf 20 setzen (= main) und im selben Zug die
// ID/Seq-Kodierung vereinheitlichen (siehe Kommentar in DataSender.cpp).
#define PACKET_VALUES  32

#define ESPNOW_MAX_BOARD_ID 255

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

struct MeasurementData {
  float forceSensor;
  float degreeSensor;
};