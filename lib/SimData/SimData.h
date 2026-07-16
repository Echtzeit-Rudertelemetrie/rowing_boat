#pragma once
#include <Arduino.h>
#include "AppTypes.h" // MeasurementPack, PACKET_VALUES
#include "Gps.h"      // GpsData
#include "Imu.h"      // ImuData

// ─────────────────────────────────────────────────────────────────────────────
// Synthetic example-data generator for the unified 132-byte MeasurementPack.
// Reuses the real wire structs, so the produced bytes are guaranteed correct.
//   gps()             -> simulated GpsData building block
//   imu()             -> simulated ImuData building block (timestamp = millis())
//   telemetry()       -> id 0: gps() in the force region, imu() in the angle region
//   dolle(boardId)    -> id 1..7: force/angle oarlock waveforms (per-board phase)
// Every source keeps its own rolling 29-bit sequence number.
// ─────────────────────────────────────────────────────────────────────────────
class SimData {
public:
    GpsData         gps();                  // simulated GPS
    ImuData         imu();                   // simulated IMU
    MeasurementPack telemetry();            // id 0: gps() + imu()
    MeasurementPack dolle(uint8_t boardId); // id 1..7

private:
    static constexpr uint8_t MAX_IDS = 8;   // 3-bit id -> 0..7
    uint32_t seq_[MAX_IDS] = {0};           // per-id rolling 29-bit sequence
    uint32_t gpsTick_      = 0;             // advances the simulated GPS motion
    uint32_t imuTick_      = 0;             // advances the simulated IMU motion

    // id in the top 3 bits, 29-bit sequence below (matches DataSender / AppTypes).
    static uint32_t packIdSeq(uint8_t id, uint32_t seq) {
        return ((uint32_t)(id & 0x07u) << 29) | (seq & 0x1FFFFFFFu);
    }
};
