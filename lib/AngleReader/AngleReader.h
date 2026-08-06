#pragma once

#include <Arduino.h>
#include <ICM_20948.h>
#include "orientation_ekf.h"
#include "AngleReaderConfig.h"
// Liefert struct MagCalibration und die Kalibrierung dieser Einheit. Die Werte
// sind einbauspezifisch und kommen deshalb aus der MAC-Tabelle statt aus einer
// Konstanten in diesem Verzeichnis.
#include "UnitIdentity.h"

enum class AngleCalibrationState : uint8_t {
    NotStarted = 0,
    WarmingUp = 1,
    Measuring = 2,
    Valid = 3,
    Failed = 4
};

struct AngleDiagnostics {
    uint32_t timestampUs = 0;
    uint32_t sequence = 0;
    float dt = 0.0f;
    float gyroRawDps[3]{};
    float gyroBiasDps[3]{};
    float gyroCorrectedDps[3]{};
    float accel[3]{};
    float accelNorm = 0.0f;
    float magRaw[3]{};
    float magCalibrated[3]{};
    float magNorm = 0.0f;
    float temperatureC = 0.0f;
    float quaternion[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float yawDeg = 0.0f;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float outputAngleDeg = 0.0f;
    bool accelValid = false;
    bool magValid = false;
    bool magFresh = false;
    bool stationary = false;
    bool gyroClipped = false;
    uint32_t invalidDtCount = 0;
    uint32_t gyroClipCount = 0;
    float dtMin = 0.0f;
    float dtMean = 0.0f;
    float dtMax = 0.0f;
    AngleCalibrationState calibrationState = AngleCalibrationState::NotStarted;
};

class AngleReader {
public:
    // Default: Kalibrierung dieser Einheit aus der MAC-Tabelle. Der explizite
    // Konstruktor bleibt fuer Tests und Sonderfaelle bestehen.
    AngleReader();
    explicit AngleReader(const MagCalibration& magCalibration);
    bool begin();
    float sampleAndCalculateAngle();
    void zeroOutputAngle();
    const AngleDiagnostics& diagnostics() const { return diagnostics_; }
    bool hardwareOk() const { return hwOk_; }

private:
    struct RunningStats {
        float mean = 0.0f;
        float m2 = 0.0f;
        uint32_t n = 0;
        void clear();
        void add(float value);
        float variance() const;
        float stddev() const;
    };

    bool calibrateGyroOffsets();
    void configureFilterNoise(const Vec3& measuredGyroVariance);
    bool readSample(Vec3& gyroV, Vec3& accelV, Vec3& magV);
    void updateStationarity(const Vec3& correctedGyro, float accelNorm, uint32_t nowMs);
    void updateOnlineBias(float dt);
    void updateMeasurementGates(float accelNorm, float magNorm, bool magFresh);
    void updateTimingStats(float dt);
    void calibrateMag(const float raw[3], float out[3]) const;
    static float clampf(float value, float low, float high);

    ICM_20948_I2C icm_;
    MagCalibration magCalibration_;
    OrientationEKF ekf_;
    Quat q_;
    bool initialized_ = false;
    bool hwOk_ = false;
    uint32_t lastUpdateUs_ = 0;
    uint32_t lastFreshMagUs_ = 0;
    float lastAngleDeg_ = 0.0f;
    float outputZeroDeg_ = 0.0f;
    float gyroBiasRadS_[3]{};
    float accelRestNorm_ = 1000.0f;
    float magRestNorm_ = 0.0f;
    bool accelValid_ = false;
    bool magValid_ = false;
    uint8_t accelGoodCount_ = 0;
    uint8_t magGoodCount_ = 0;

    static constexpr uint16_t kStationaryWindow = AngleConfig::STATIONARY_WINDOW_SAMPLES;
    float gyroWindow_[kStationaryWindow][3]{};
    uint16_t gyroWindowIndex_ = 0;
    uint16_t gyroWindowCount_ = 0;
    float gyroWindowSum_[3]{};
    float gyroWindowSumSq_[3]{};
    uint32_t stationaryCandidateSinceMs_ = 0;
    bool stationary_ = false;

    float latestRawGyroRadS_[3]{};
    AngleDiagnostics diagnostics_{};
    double dtSum_ = 0.0;
    uint32_t dtCount_ = 0;
};
