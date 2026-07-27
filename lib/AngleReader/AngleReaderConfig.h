#pragma once

// Central configuration for the XIAO ESP32S3 / ICM-20948 orientation path.
// Values marked "tune from hardware logs" are conservative starting points,
// not claimed sensor specifications.
namespace AngleConfig {

constexpr unsigned long SAMPLE_INTERVAL_US = 10000UL; // application timer: 100 Hz
constexpr float MIN_VALID_DT_S = 0.004f;
constexpr float MAX_VALID_DT_S = 0.030f;

// ICM-20948 ODR = 1.125 kHz / (1 + divider) => about 102.3 Hz.
constexpr uint16_t ACCEL_RATE_DIVIDER = 10;
constexpr uint8_t GYRO_RATE_DIVIDER = 10;

// Boot calibration. A failed attempt is retried; after all retries the device
// remains operational in a clearly reported degraded state with zero boot bias.
constexpr uint32_t BOOT_WARMUP_MS = 1500;
constexpr uint32_t BOOT_CALIBRATION_MS = 4000;
constexpr uint8_t BOOT_CALIBRATION_ATTEMPTS = 3;
constexpr uint16_t BOOT_MIN_SAMPLES = 250;
constexpr float BOOT_MAX_GYRO_MEAN_DPS = 0.5f;
constexpr float BOOT_MAX_GYRO_STD_DPS = 0.20f;
constexpr float BOOT_MAX_ACCEL_STD_FRACTION = 0.010f;
constexpr float BOOT_ACCEL_NORM_MIN_MG = 700.0f;
constexpr float BOOT_ACCEL_NORM_MAX_MG = 1300.0f;

// Stationarity detector: all instantaneous conditions must remain true for the
// hold period. The low rate threshold deliberately rejects slow real motion.
constexpr uint16_t STATIONARY_WINDOW_SAMPLES = 50; // 0.5 s at 100 Hz
constexpr uint32_t STATIONARY_HOLD_MS = 1200;
constexpr float STATIONARY_MAX_RATE_DPS = 0.35f;
constexpr float STATIONARY_MAX_MEAN_RATE_DPS = 0.05f;
constexpr float STATIONARY_MAX_GYRO_STD_DPS = 0.12f;
constexpr float STATIONARY_ACCEL_TOLERANCE = 0.05f;

// Online bias is updated only while stationary. The slew limit prevents one
// misclassified interval from corrupting calibration.
constexpr float ONLINE_BIAS_TIME_CONSTANT_S = 3.0f;
constexpr float ONLINE_BIAS_MAX_SLEW_DPS_PER_S = 0.05f;

// Measurement gates include hysteresis. A sensor must remain good for several
// samples before it is re-enabled, avoiding correction chatter.
constexpr float ACCEL_VALID_ENTER_FRACTION = 0.08f;
constexpr float ACCEL_VALID_EXIT_FRACTION = 0.14f;
constexpr uint8_t ACCEL_VALID_CONFIRM_SAMPLES = 8;
constexpr float MAG_VALID_ENTER_FRACTION = 0.18f;
constexpr float MAG_VALID_EXIT_FRACTION = 0.30f;
constexpr uint8_t MAG_VALID_CONFIRM_SAMPLES = 12;
constexpr float MAG_NORM_MIN_UT = 10.0f;
// Until a proper hard-/soft-iron calibration is installed, mounted assemblies
// can show a large raw offset. Ratio gating remains active; the wider absolute
// ceiling merely permits collecting and validating such data.
constexpr float MAG_NORM_MAX_UT = 200.0f;
// The AK09916 in the installed ICM-20948 assemblies produces fresh samples at
// roughly 7-10 Hz. Keep the last vector long enough to bridge that measured
// interval; the plausibility and norm gates still reject disturbed readings.
constexpr uint32_t MAG_MAX_STALE_US = 250000UL;

// EKF tuning: measured white sensor noise is only one component. These floors
// also represent model error, gyro bias drift, dynamic acceleration and local
// magnetic disturbance. Tune from repeatable hardware tests.
constexpr float GYRO_PROCESS_VARIANCE_FLOOR = 1.0e-4f; // (rad/s)^2
constexpr float GYRO_PROCESS_VARIANCE_CEILING = 5.0e-2f;
constexpr float ACCEL_MEAS_VARIANCE_FLOOR = 2.5e-3f;   // normalized vector
constexpr float ACCEL_MEAS_VARIANCE_CEILING = 2.5e-1f;
constexpr float MAG_MEAS_VARIANCE_FLOOR = 5.0e-3f;     // normalized vector
constexpr float MAG_MEAS_VARIANCE_CEILING = 5.0e-1f;
constexpr float REJECTED_MEAS_VARIANCE = 1.0e3f;

constexpr float GYRO_CLIP_WARNING_DPS = 490.0f;

// No trustworthy calibration provenance exists. Neutral values are safer than
// silently applying the inherited, installation-specific constants. Replace
// these values with output from scripts/calibrate_magnetometer.py.
constexpr bool MAG_CALIBRATION_VERIFIED = false;
constexpr float MAG_A[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};
constexpr float MAG_B[3] = {0.0f, 0.0f, 0.0f};

} // namespace AngleConfig
