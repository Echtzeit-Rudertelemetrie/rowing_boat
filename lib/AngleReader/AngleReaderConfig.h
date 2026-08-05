#pragma once

// Central configuration for the XIAO ESP32S3 / ICM-20948 orientation path.
// Values marked "tune from hardware logs" are conservative starting points,
// not claimed sensor specifications.
namespace AngleConfig {

// 200 Hz filter rate (2026-07-29). Gyro and accel genuinely update faster than
// 100 Hz and the stroke reaches 276 deg/s, so the extra samples carry real
// information. The AK09916 does not: it delivers fresh data at roughly 7-10 Hz
// in this assembly, so the magnetometer contributes on its own schedule and the
// staleness gate (MAG_MAX_STALE_US) keeps it honest.
//
// What must NOT follow this rate is the radio: the phone app hardcodes 10 ms
// per sample (BluetoothPacket.sampleIntervalMs), so the sender decimates by two
// and keeps feeding packets at 100 Hz. See EspNow_sender_App::handleEvent.
constexpr unsigned long SAMPLE_INTERVAL_US = 5000UL; // application timer: 200 Hz
// Nominal dt is now 5 ms, so the old 4 ms floor would reject ordinary jitter.
constexpr float MIN_VALID_DT_S = 0.002f;
constexpr float MAX_VALID_DT_S = 0.030f;

// ICM-20948 ODR = 1.125 kHz / (1 + divider) => about 225 Hz, comfortably above
// the 200 Hz sample rate so every tick finds a fresh gyro/accel reading.
constexpr uint16_t ACCEL_RATE_DIVIDER = 4;
constexpr uint8_t GYRO_RATE_DIVIDER = 4;

// Boot calibration. A failed attempt is retried; after all retries the device
// remains operational in a clearly reported degraded state with zero boot bias.
constexpr uint32_t BOOT_WARMUP_MS = 1500;
constexpr uint32_t BOOT_CALIBRATION_MS = 4000;
constexpr uint8_t BOOT_CALIBRATION_ATTEMPTS = 3;
constexpr uint16_t BOOT_MIN_SAMPLES = 250;
// The mean gate bounds the zero-rate offset being measured, NOT motion —
// motion is caught by the std gates below. Keeping it at the old 0.5 dps
// rejected a demonstrably motionless board 14:c1:9f:c5:c8:50 (2026-07-29):
// std 0.060 dps against a 0.20 limit, yet mean -1.126 dps, reproduced across
// attempts to within 0.002 dps — a constant offset, not rotation. It could
// therefore never calibrate and always ran on zero bias, drifting 67 deg/min.
// The ICM-20948 specifies +-5 dps zero-rate offset, so 2.0 dps still leaves
// headroom while staying far below any rotation a person would call "still".
constexpr float BOOT_MAX_GYRO_MEAN_DPS = 2.0f;
constexpr float BOOT_MAX_GYRO_STD_DPS = 0.20f;
constexpr float BOOT_MAX_ACCEL_STD_FRACTION = 0.010f;
constexpr float BOOT_ACCEL_NORM_MIN_MG = 700.0f;
constexpr float BOOT_ACCEL_NORM_MAX_MG = 1300.0f;

// Stationarity detector: all instantaneous conditions must remain true for the
// hold period. The low rate threshold deliberately rejects slow real motion.
constexpr uint16_t STATIONARY_WINDOW_SAMPLES = 100; // 0.5 s at 200 Hz
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

// ACTIVE: board 14:c1:9f:c5:c8:50 (prototype). Generated by
// scripts/calibrate_magnetometer.py from logs/oarlock_mag_05.csv (2026-07-29),
// residual 5.04%, field magnitude 47.7 uT against Earth's ~48 in Germany — the
// magnitude landing on the true field is the strongest single indicator that
// the fit is sound.
//
// Keep calibration recordings SHORT. Four attempts on this unit: 120 s runs
// gave 8.76%, 12.05% and 10.48%, a 60 s run gave 5.04%. The sphere centre
// drifts during a recording (32 uT with a loose USB cable, 16 uT with the cable
// taped down, 14 uT over 60 s), so the hard iron on this assembly is not fully
// stable and a longer tumble simply averages over more of that drift. Securing
// the cable helps; shortening the run helps more. Coverage was never the
// limit — the worst run had the best coverage.
//
// Hard/soft iron is installation-specific: never build these constants for a
// different unit. Measured consequence on board 14:c1:9f:c6:fa:98 (2026-07-29):
// the mag vector rotated 5.5 deg while the body rotated 88.9 deg, so the EKF
// read the uncancelled body-fixed residual as an absolute reference and decayed
// roll back to the boot orientation with a ~35 s time constant.
//
// Board 14:c1:9f:c6:fa:98 has no usable calibration — three recordings, one at
// 97% spherical coverage, all fail (radius 31.8 uT, scatter 37.1%, residual
// 35.9%; raw magnitude ~110 uT vs ~85 uT here). Same room and procedure, so it
// is a hardware fault on that assembly. When building for that unit, disable
// the magnetometer instead:
//   constexpr bool MAG_CALIBRATION_VERIFIED = false;
//   MAG_A = identity, MAG_B = {0, 0, 0}
// Its own 97%-coverage fit was tried and does remove the pull-back, but it
// contributes only ~0.7 deg/min of correction — harmless, not useful:
//   MAG_A {2.77623467f, 0.697274236f, -0.86762364f},
//         {0.697274236f, 0.482282293f, -0.191071711f},
//         {-0.86762364f, -0.191071711f, 1.4394058f}
//   MAG_B {77.3478971f, 1.94298809f, -71.6231525f}
constexpr bool MAG_CALIBRATION_VERIFIED = true;
constexpr float MAG_A[3][3] = {
    {1.17896825f, 0.113278611f, -0.0805388595f},
    {0.113278611f, 0.961995023f, 0.0940220557f},
    {-0.0805388595f, 0.0940220557f, 1.04792101f},
};
constexpr float MAG_B[3] = {-61.9654816f, -49.1350037f, 46.512721f};

} // namespace AngleConfig
