#pragma once
#include <Arduino.h>
#include "AppTypes.h" // MeasurementPack, PACKET_VALUES
#include "Gps.h"      // GpsData
#include "Imu.h"      // ImuData

// ─────────────────────────────────────────────────────────────────────────────
// Synthetic example-data generator for the unified 84-byte MeasurementPack.
// Reuses the real wire structs, so the produced bytes are guaranteed correct.
//   telemetry()       -> id 0: GpsData in the force region, ImuData in the angle
//                              region (drifts/oscillates a bit each call)
//   dolle(boardId)    -> id 1..15: force/angle oarlock waveforms (per-board phase)
// Every source keeps its own rolling 28-bit sequence number.
// ─────────────────────────────────────────────────────────────────────────────
class SimData {
public:
    MeasurementPack telemetry();            // id 0
    MeasurementPack dolle(uint8_t boardId); // id 1..15

private:
    uint32_t seq_[16] = {0};   // per-id rolling 28-bit sequence
    uint32_t tick_    = 0;      // advances the simulated GPS/IMU motion

    static uint32_t packIdSeq(uint8_t id, uint32_t seq) {
        return ((uint32_t)(id & 0x0F) << 28) | (seq & 0x0FFFFFFFu);
    }
};
