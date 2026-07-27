#pragma once
#include <Arduino.h>
#include "AppTypes.h" // MeasurementPack, PACKET_VALUES
#include "Gps.h"
#include "Imu.h"      // ImuData

// ─────────────────────────────────────────────────────────────────────────────
// Synthetic example-data generator for the unified 36-byte MeasurementPack.
// Reuses the real wire structs, so the produced bytes are guaranteed correct.
//   gps()             -> simulated GpsData building block
//   imu()             -> simulated ImuData building block (timestamp = millis())
//   telemetry()       -> id 0: gps() in the force region, imu() in the angle region
//   dolle(boardId)    -> id 1..15: force/angle oarlock waveforms (per-board phase)
// Every source keeps its own rolling 28-bit sequence number.
// ─────────────────────────────────────────────────────────────────────────────
class SimData {
public:
    GpsData         gps();                  // simulated GPS
    ImuData         imu();                   // simulated IMU
    MeasurementPack telemetry();            // id 0: gps() + imu()
    MeasurementPack dolle(uint8_t boardId); // id 1..15

private:
    // id indexes seq_ directly, so this must cover the full ID range.
    static constexpr uint8_t MAX_IDS = IDSEQ_ID_MASK + 1;
    uint32_t seq_[MAX_IDS] = {0};           // per-id rolling 28-bit sequence
    uint32_t gpsTick_      = 0;             // advances the simulated GPS motion
    uint32_t imuTick_      = 0;             // advances the simulated IMU motion
};
