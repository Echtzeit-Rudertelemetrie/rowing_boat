#include <Arduino.h>
#include <Wire.h>
#include "AngleReader.h"

namespace {
constexpr uint32_t kSamplePeriodUs = 10000U;
constexpr uint8_t kCsvDecimation = 2; // 50 Hz keeps full rows below 460800 baud
constexpr const char* kFirmwareId = "xiao_s3_angle_diag_v1";

AngleReader angleReader;
uint32_t nextSampleUs = 0;
uint32_t csvCounter = 0;

void printHeader() {
    Serial.println(
        "timestamp_us,sequence_number,dt_s,"
        "gyro_raw_x_dps,gyro_raw_y_dps,gyro_raw_z_dps,"
        "gyro_bias_x_dps,gyro_bias_y_dps,gyro_bias_z_dps,"
        "gyro_corrected_x_dps,gyro_corrected_y_dps,gyro_corrected_z_dps,"
        "accel_x,accel_y,accel_z,accel_norm,"
        "mag_raw_x,mag_raw_y,mag_raw_z,"
        "mag_calibrated_x,mag_calibrated_y,mag_calibrated_z,mag_norm,"
        "quaternion_w,quaternion_x,quaternion_y,quaternion_z,"
        "yaw_deg,pitch_deg,roll_deg,output_angle_deg,"
        "accel_valid,mag_valid,mag_fresh,stationary,"
        "queue_depth,dropped_events,calibration_state,temperature_c,"
        "invalid_dt_count,gyro_clip_count");
}

void printCsv(const AngleDiagnostics& d) {
    Serial.printf(
        "%lu,%lu,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%.7f,%.7f,%.7f,%.7f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%d,%d,%d,%d,0,0,%u,%.3f,%lu,%lu\n",
        static_cast<unsigned long>(d.timestampUs),
        static_cast<unsigned long>(d.sequence), d.dt,
        d.gyroRawDps[0], d.gyroRawDps[1], d.gyroRawDps[2],
        d.gyroBiasDps[0], d.gyroBiasDps[1], d.gyroBiasDps[2],
        d.gyroCorrectedDps[0], d.gyroCorrectedDps[1], d.gyroCorrectedDps[2],
        d.accel[0], d.accel[1], d.accel[2], d.accelNorm,
        d.magRaw[0], d.magRaw[1], d.magRaw[2],
        d.magCalibrated[0], d.magCalibrated[1], d.magCalibrated[2], d.magNorm,
        d.quaternion[0], d.quaternion[1], d.quaternion[2], d.quaternion[3],
        d.yawDeg, d.pitchDeg, d.rollDeg, d.outputAngleDeg,
        d.accelValid ? 1 : 0, d.magValid ? 1 : 0, d.magFresh ? 1 : 0,
        d.stationary ? 1 : 0,
        static_cast<unsigned>(d.calibrationState), d.temperatureC,
        static_cast<unsigned long>(d.invalidDtCount),
        static_cast<unsigned long>(d.gyroClipCount));
}
} // namespace

void setup() {
    Serial.begin(460800);
    delay(500);
    Serial.printf("# firmware_id=%s\n", kFirmwareId);
    Serial.println("# commands: z=set current roll as output zero, h=print help");
    Serial.println("# boot: keep the complete assembly motionless until ANGLE_CAL,SUCCESS");
    Wire.begin();
    Wire.setClock(400000);
    if (!angleReader.begin()) {
        Serial.println("# FATAL: AngleReader initialization failed");
        while (true) delay(1000);
    }
    printHeader();
    nextSampleUs = micros();
}

void loop() {
    if (Serial.available()) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'z' || command == 'Z') {
            angleReader.zeroOutputAngle();
            Serial.println("# ZERO_APPLIED");
        } else if (command == 'h' || command == 'H') {
            Serial.println("# z: current physical position becomes output angle 0 degrees");
        }
    }

    const uint32_t now = micros();
    if (static_cast<int32_t>(now - nextSampleUs) < 0) return;
    // Advance by the fixed period. If output ever falls far behind, resynchronize
    // instead of emitting a burst of stale-time calls.
    nextSampleUs += kSamplePeriodUs;
    if (static_cast<int32_t>(now - nextSampleUs) > static_cast<int32_t>(5 * kSamplePeriodUs))
        nextSampleUs = now + kSamplePeriodUs;

    angleReader.sampleAndCalculateAngle();
    if (++csvCounter >= kCsvDecimation) {
        csvCounter = 0;
        printCsv(angleReader.diagnostics());
    }
}
