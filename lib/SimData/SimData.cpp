#include "SimData.h"
#include <math.h>
#include <string.h>

GpsData SimData::gps() {
    const uint32_t t = gpsTick_++;
    GpsData g{};
    g.lat_e6      = 52500000;                       // 52.500000°
    g.lon_e6      = 13400000 + (int32_t)(t * 9);    // ~0.6 m east per tick
    g.speed_cms   = (uint16_t)(400 + (t % 3) * 100);// 4..6 m/s
    g.course_cdeg = 9000;                           // heading east
    g.satellites = 9;
    g.valid      = true;
    return g;
}

ImuData SimData::imu() {
    const uint32_t t = imuTick_++;
    ImuData imu{};
    imu.acc_x_mg     = (int16_t)(204.0f * sinf(t * 0.5f));  // stroke accel ±2 m/s^2
    imu.acc_y_mg     = (int16_t)(31.0f * cosf(t * 0.5f));
    imu.acc_z_mg     = 1000;                                 // gravity on Z
    imu.pitch_cdeg   = (int16_t)(300.0f * sinf(t * 0.5f));   // boat pitch ±3°
    imu.timestamp_ms = millis();
    return imu;
}

MeasurementPack SimData::telemetry() {
    const GpsData g   = gps();
    const ImuData imu = this->imu();

    MeasurementPack p{};
    p.espIdAndSeqenceNum = packIdSeq(0, seq_[0]++);   // id 0 = telemetry
    memcpy(p.force_values, &g,   sizeof(g));          // GPS -> force region
    memcpy(p.angle_values, &imu, sizeof(imu));        // IMU -> angle region
    return p;
}

MeasurementPack SimData::dolle(uint8_t boardId) {
    if (boardId < 1 || boardId > IDSEQ_ID_MASK) boardId = 1;
    const uint32_t s = seq_[boardId]++;

    MeasurementPack p{};
    p.espIdAndSeqenceNum = packIdSeq(boardId, s);     // id = boardId

    // PACKET_VALUES samples span one drive/recovery stroke. Phase-shift per board
    // so the oarlocks look distinct (e.g. port vs starboard), plus a slow drift.
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
