#include "SimData.h"
#include <math.h>
#include <string.h>

MeasurementPack SimData::telemetry() {
    const uint32_t t = tick_++;

    // --- GPS: drift east along a rowing course (near Berlin) ---
    GpsData g{};
    g.lat        = 52500000;                       // 52.500000°
    g.lon        = 13400000 + (int32_t)(t * 9);    // ~0.6 m east per tick
    g.speed_mps  = (int16_t)(4 + (t % 3));         // 4..6 m/s (int16: whole m/s)
    g.course_deg = 90;                             // heading east
    g.satellites = 9;
    g.valid      = true;

    // --- IMU: gravity on Z + a rowing-stroke oscillation on X ---
    ImuData imu{};
    imu.acc_x        = 2.0f * sinf(t * 0.5f);      // stroke accel ±2 m/s^2
    imu.acc_y        = 0.3f * cosf(t * 0.5f);
    imu.acc_z        = 9.81f;
    imu.timestamp_ms = millis();

    MeasurementPack p{};
    p.idAndSeq = packIdSeq(0, seq_[0]++);          // id 0 = telemetry
    memcpy(p.force_values, &g,   sizeof(g));       // GPS -> force region
    memcpy(p.angle_values, &imu, sizeof(imu));     // IMU -> angle region
    return p;
}

MeasurementPack SimData::dolle(uint8_t boardId) {
    if (boardId < 1 || boardId > 15) boardId = 1;
    const uint32_t s = seq_[boardId]++;

    MeasurementPack p{};
    p.idAndSeq = packIdSeq(boardId, s);            // id = boardId

    // 20 samples span one drive/recovery stroke. Phase-shift per board so the two
    // oarlocks look distinct (e.g. port vs starboard), plus a slow drift per packet.
    const float boardPhase = (boardId - 1) * 0.6f;
    const float pktPhase   = s * 0.3f;
    for (uint8_t i = 0; i < PACKET_VALUES; i++) {
        const float u = (float)i / (PACKET_VALUES - 1);                 // 0..1
        // force: half-sine drive pulse, ~0..600 N mapped onto 0..1000 N full scale
        const float forceN = 600.0f * fmaxf(0.0f, sinf((u + boardPhase + pktPhase) * PI));
        p.force_values[i]  = (uint16_t)(forceN / 1000.0f * 65535.0f);
        // angle: sweep catch(-60°)..finish(+30°) mapped onto -90..+90° full scale
        const float angleDeg = -60.0f + 90.0f * u;
        p.angle_values[i]    = (uint16_t)((angleDeg + 90.0f) / 180.0f * 65535.0f);
    }
    return p;
}
