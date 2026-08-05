#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <climits>
#include "AppTypes.h"

// ESP-NOW broadcast has no application-level acknowledgement. Re-sending the
// same sequence number caused duplicate and out-of-order packets at the phone,
// which made its time axis jump backwards. Send each 100-Hz sample packet once;
// real loss is detected by the sequence number.
constexpr std::uint8_t PACKET_RETRIES = 1;

// Die ESP-ID ist keine Konstante mehr: sie kommt aus der MAC des Boards, siehe
// OarlockIdentity.h. Ein fest einkompiliertes ESP_ID = 1 bedeutete pro Dolle
// einen eigenen Build, und zwei Boards mit demselben Build belegten dieselbe
// ID -- das hat schon einen Systemtest gekostet.

// ── Quantisierungs-Spannen ───────────────────────────────────────────────────
// Diese Grenzen sind NICHT nur ein Clamp, sondern der MASSSTAB: quantize()
// bildet [MIN, MAX] auf die 65536 uint16-Codes ab. Wer sie aendert, aendert die
// Bedeutung jedes gesendeten Werts.
//
// TODO/ABSTIMMUNG: Der Empfaenger MUSS mit denselben Spannen dequantisieren —
// sonst sind die Werte still falsch (kein Laengencheck faengt das ab):
//     force_N   = code / 65535.0 * (FORCE_MAX_N - FORCE_MIN_N) + FORCE_MIN_N
//               = code / 65535.0 * 1000.0
//     angle_deg = code / 65535.0 * (ANGLE_MAX_DEG - ANGLE_MIN_DEG) + ANGLE_MIN_DEG
//               = code / 65535.0 * 360.0 - 180.0
// Gehoert zusammen mit PACKET_VALUES und der ID/Seq-Kodierung abgestimmt
// (siehe AppTypes.h und den TODO-Block in DataSender.cpp) und im README
// dokumentiert -> Abschnitt "Paketformat & Quantisierung".
constexpr float FORCE_MIN_N   = 0.0f;
constexpr float FORCE_MAX_N   = 1000.0f;
// +/-180 deg deckt den vollen Ausgabebereich von quatToEulerDeg() ab (Roll aus
// atan2 liegt in (-180, +180]). Aufloesung 360/65535 = 0.0055 deg.
constexpr float ANGLE_MIN_DEG = -180.0f;
constexpr float ANGLE_MAX_DEG =  180.0f;

// Kehrwerte zur Compile-Zeit: Division hat auf der LX7-FPU keinen eigenen
// Befehl (mehrzyklige Reziprok-Sequenz) — quantize() multipliziert damit
// stattdessen, 2x pro 100-Hz-Tick.
constexpr float FORCE_INV_SPAN = 1.0f / (FORCE_MAX_N - FORCE_MIN_N);
constexpr float ANGLE_INV_SPAN = 1.0f / (ANGLE_MAX_DEG - ANGLE_MIN_DEG);

class DataSender {
public:
    explicit DataSender(MeasurementData& data);

    void espnow_init_sender();
    void sendData();

private:
    MeasurementData* data_{nullptr};
};
