#include "AngleReader.h"
#include <math.h>

namespace {
constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.29577951f;

Quat identityQuaternion() {
    Quat q;
    q.m[0][0] = 1.0f;
    return q;
}

MagCalibration oarlockMagCalibration() {
    MagCalibration calibration{};
    calibration.verified = AngleConfig::MAG_CALIBRATION_VERIFIED;
    for (int row = 0; row < 3; ++row) {
        calibration.offset[row] = AngleConfig::MAG_B[row];
        for (int column = 0; column < 3; ++column) {
            calibration.matrix[row][column] = AngleConfig::MAG_A[row][column];
        }
    }
    return calibration;
}
} // namespace

void AngleReader::RunningStats::clear() {
    mean = 0.0f;
    m2 = 0.0f;
    n = 0;
}

void AngleReader::RunningStats::add(float value) {
    ++n;
    const float delta = value - mean;
    mean += delta / static_cast<float>(n);
    m2 += delta * (value - mean);
}

float AngleReader::RunningStats::variance() const {
    return n > 1 ? fmaxf(0.0f, m2 / static_cast<float>(n - 1)) : 0.0f;
}

float AngleReader::RunningStats::stddev() const {
    return sqrtf(variance());
}

AngleReader::AngleReader()
    : AngleReader(oarlockMagCalibration()) {
}

AngleReader::AngleReader(const MagCalibration& magCalibration)
    : ekf_(AngleConfig::GYRO_PROCESS_VARIANCE_FLOOR,
           AngleConfig::ACCEL_MEAS_VARIANCE_FLOOR,
           AngleConfig::MAG_MEAS_VARIANCE_FLOOR),
      q_(identityQuaternion()),
      magCalibration_(magCalibration) {
}

float AngleReader::clampf(float value, float low, float high) {
    return fminf(high, fmaxf(low, value));
}

bool AngleReader::begin() {
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
        if (attempt > 0) delay(100);
        ok = (icm_.begin(Wire, true) == ICM_20948_Stat_Ok);
    }
    if (!ok) {
        Serial.print("ANGLE_ERROR,ICM20948_NOT_FOUND,");
        Serial.println(icm_.statusString());
        return false;
    }

    ICM_20948_fss_t fss;
    fss.a = gpm8;
    fss.g = dps500;
    icm_.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);

    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d23bw9_n34bw4;
    dlp.g = gyr_d23bw9_n35bw9;
    icm_.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlp);
    icm_.enableDLPF(ICM_20948_Internal_Acc, true);
    icm_.enableDLPF(ICM_20948_Internal_Gyr, true);

    ICM_20948_smplrt_t rate;
    rate.a = AngleConfig::ACCEL_RATE_DIVIDER;
    rate.g = AngleConfig::GYRO_RATE_DIVIDER;
    const ICM_20948_Status_e rateStatus =
        icm_.setSampleRate(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, rate);
    Serial.printf("ANGLE_CONFIG,odr_hz=102.3,rate_status=%d,mag_cal_verified=%d\n",
                  static_cast<int>(rateStatus), magCalibration_.verified ? 1 : 0);

    hwOk_ = true;
    if (!calibrateGyroOffsets()) {
        // Continue in degraded mode. The EKF and later safe stationary periods can
        // recover bias, while diagnostics clearly expose the failed boot state.
        for (float& bias : gyroBiasRadS_) bias = 0.0f;
        diagnostics_.calibrationState = AngleCalibrationState::Failed;
        Serial.println("ANGLE_WARNING,BOOT_CALIBRATION_FAILED,using_zero_bias");
    }
    return true;
}

bool AngleReader::calibrateGyroOffsets() {
    diagnostics_.calibrationState = AngleCalibrationState::WarmingUp;
    Serial.printf("ANGLE_CAL,WARMUP,%lu_ms,do_not_move\n",
                  static_cast<unsigned long>(AngleConfig::BOOT_WARMUP_MS));
    delay(AngleConfig::BOOT_WARMUP_MS);

    for (uint8_t attempt = 1; attempt <= AngleConfig::BOOT_CALIBRATION_ATTEMPTS; ++attempt) {
        diagnostics_.calibrationState = AngleCalibrationState::Measuring;
        Serial.printf("ANGLE_CAL,MEASURING,attempt=%u,duration_ms=%lu\n",
                      attempt, static_cast<unsigned long>(AngleConfig::BOOT_CALIBRATION_MS));

        RunningStats gyro[3];
        RunningStats accelNorm;
        const uint32_t startMs = millis();
        while (static_cast<uint32_t>(millis() - startMs) < AngleConfig::BOOT_CALIBRATION_MS) {
            if (icm_.dataReady()) {
                icm_.getAGMT();
                gyro[0].add(icm_.gyrX());
                gyro[1].add(icm_.gyrY());
                gyro[2].add(icm_.gyrZ());
                const float ax = icm_.accX(), ay = icm_.accY(), az = icm_.accZ();
                accelNorm.add(sqrtf(ax * ax + ay * ay + az * az));
            }
            delay(1);
        }

        const float accelStdFraction =
            accelNorm.mean > 1.0f ? accelNorm.stddev() / accelNorm.mean : 1.0f;
        bool valid = gyro[0].n >= AngleConfig::BOOT_MIN_SAMPLES &&
                     accelNorm.mean >= AngleConfig::BOOT_ACCEL_NORM_MIN_MG &&
                     accelNorm.mean <= AngleConfig::BOOT_ACCEL_NORM_MAX_MG &&
                     accelStdFraction <= AngleConfig::BOOT_MAX_ACCEL_STD_FRACTION;
        for (int i = 0; i < 3; ++i) {
            valid = valid &&
                    fabsf(gyro[i].mean) <= AngleConfig::BOOT_MAX_GYRO_MEAN_DPS &&
                    gyro[i].stddev() <= AngleConfig::BOOT_MAX_GYRO_STD_DPS;
        }

        Serial.printf(
            "ANGLE_CAL,QUALITY,attempt=%u,samples=%lu,"
            "mean_dps=%.5f|%.5f|%.5f,std_dps=%.5f|%.5f|%.5f,"
            "accel_mean_mg=%.2f,accel_std_fraction=%.6f,result=%s\n",
            attempt, static_cast<unsigned long>(gyro[0].n),
            gyro[0].mean, gyro[1].mean, gyro[2].mean,
            gyro[0].stddev(), gyro[1].stddev(), gyro[2].stddev(),
            accelNorm.mean, accelStdFraction, valid ? "VALID" : "MOVEMENT");

        if (valid) {
            Vec3 measuredGyroVar;
            for (int i = 0; i < 3; ++i) {
                gyroBiasRadS_[i] = gyro[i].mean * kDegToRad;
                measuredGyroVar.m[i][0] = gyro[i].variance() * kDegToRad * kDegToRad;
            }
            accelRestNorm_ = accelNorm.mean;
            configureFilterNoise(measuredGyroVar);
            diagnostics_.calibrationState = AngleCalibrationState::Valid;
            Serial.printf("ANGLE_CAL,SUCCESS,bias_dps=%.6f|%.6f|%.6f,accel_rest_mg=%.3f\n",
                          gyro[0].mean, gyro[1].mean, gyro[2].mean, accelRestNorm_);
            return true;
        }
        if (attempt < AngleConfig::BOOT_CALIBRATION_ATTEMPTS) {
            Serial.println("ANGLE_CAL,RETRY,keep_device_completely_still");
            delay(1000);
        }
    }
    diagnostics_.calibrationState = AngleCalibrationState::Failed;
    return false;
}

void AngleReader::configureFilterNoise(const Vec3& measuredGyroVariance) {
    Vec3 gyroQ, accelR, magR;
    for (int i = 0; i < 3; ++i) {
        gyroQ.m[i][0] = clampf(measuredGyroVariance.m[i][0],
                               AngleConfig::GYRO_PROCESS_VARIANCE_FLOOR,
                               AngleConfig::GYRO_PROCESS_VARIANCE_CEILING);
        accelR.m[i][0] = AngleConfig::ACCEL_MEAS_VARIANCE_FLOOR;
        magR.m[i][0] = AngleConfig::MAG_MEAS_VARIANCE_FLOOR;
    }
    ekf_.setNoise(gyroQ, accelR, magR);
}

void AngleReader::calibrateMag(const float raw[3], float out[3]) const {
    float centered[3] = {
        raw[0] - magCalibration_.offset[0],
        raw[1] - magCalibration_.offset[1],
        raw[2] - magCalibration_.offset[2]
    };
    for (int row = 0; row < 3; ++row) {
        out[row] = magCalibration_.matrix[row][0] * centered[0] +
                   magCalibration_.matrix[row][1] * centered[1] +
                   magCalibration_.matrix[row][2] * centered[2];
    }
}

bool AngleReader::readSample(Vec3& gyroV, Vec3& accelV, Vec3& magV) {
    if (!hwOk_ || !icm_.dataReady()) return false;
    icm_.getAGMT();

    const float rawGyroDps[3] = {icm_.gyrX(), icm_.gyrY(), icm_.gyrZ()};
    const float accelNative[3] = {icm_.accX(), icm_.accY(), icm_.accZ()};
    const float magAligned[3] = {icm_.magX(), -icm_.magY(), -icm_.magZ()};
    float magCal[3];
    calibrateMag(magAligned, magCal);

    for (int i = 0; i < 3; ++i) {
        latestRawGyroRadS_[i] = rawGyroDps[i] * kDegToRad;
        diagnostics_.gyroRawDps[i] = rawGyroDps[i];
        diagnostics_.gyroBiasDps[i] = gyroBiasRadS_[i] * kRadToDeg;
        diagnostics_.accel[i] = accelNative[i];
        diagnostics_.magRaw[i] = magAligned[i];
        diagnostics_.magCalibrated[i] = magCal[i];
    }

    const float gx = latestRawGyroRadS_[0] - gyroBiasRadS_[0];
    const float gy = latestRawGyroRadS_[1] - gyroBiasRadS_[1];
    const float gz = latestRawGyroRadS_[2] - gyroBiasRadS_[2];

    // Physical mounting -> EKF frame. Sensor X is the oarlock axis and is already
    // EKF X, so the frames coincide; output remains roll.
    gyroV = vec3(gx, gy, gz);
    accelV = vec3(accelNative[0], accelNative[1], accelNative[2]);
    magV = vec3(magCal[0], magCal[1], magCal[2]);

    diagnostics_.gyroCorrectedDps[0] = gyroV.m[0][0] * kRadToDeg;
    diagnostics_.gyroCorrectedDps[1] = gyroV.m[1][0] * kRadToDeg;
    diagnostics_.gyroCorrectedDps[2] = gyroV.m[2][0] * kRadToDeg;
    diagnostics_.accelNorm = norm3(accelV);
    diagnostics_.magNorm = norm3(magV);
    diagnostics_.temperatureC = icm_.temp();

    // AK09916 ST1.DRDY bit 0 and ST2.HOFL bit 3 are part of the nine-byte
    // external-sensor shadow read by getAGMT().
    const bool drdy = (icm_.agmt.magStat1 & 0x01u) != 0;
    const bool overflow = (icm_.agmt.magStat2 & 0x08u) != 0;
    diagnostics_.magFresh = drdy && !overflow;
    if (diagnostics_.magFresh) lastFreshMagUs_ = micros();

    diagnostics_.gyroClipped = fabsf(rawGyroDps[0]) >= AngleConfig::GYRO_CLIP_WARNING_DPS ||
                               fabsf(rawGyroDps[1]) >= AngleConfig::GYRO_CLIP_WARNING_DPS ||
                               fabsf(rawGyroDps[2]) >= AngleConfig::GYRO_CLIP_WARNING_DPS;
    if (diagnostics_.gyroClipped) ++diagnostics_.gyroClipCount;
    return true;
}

void AngleReader::updateStationarity(const Vec3& gyro, float accelNorm, uint32_t nowMs) {
    float correctedDps[3];
    for (int axis = 0; axis < 3; ++axis) correctedDps[axis] = gyro.m[axis][0] * kRadToDeg;

    if (gyroWindowCount_ == kStationaryWindow) {
        for (int axis = 0; axis < 3; ++axis) {
            const float old = gyroWindow_[gyroWindowIndex_][axis];
            gyroWindowSum_[axis] -= old;
            gyroWindowSumSq_[axis] -= old * old;
        }
    } else {
        ++gyroWindowCount_;
    }
    for (int axis = 0; axis < 3; ++axis) {
        gyroWindow_[gyroWindowIndex_][axis] = correctedDps[axis];
        gyroWindowSum_[axis] += correctedDps[axis];
        gyroWindowSumSq_[axis] += correctedDps[axis] * correctedDps[axis];
    }
    gyroWindowIndex_ = (gyroWindowIndex_ + 1) % kStationaryWindow;

    const float rateNorm = sqrtf(correctedDps[0] * correctedDps[0] +
                                 correctedDps[1] * correctedDps[1] +
                                 correctedDps[2] * correctedDps[2]);
    const float accelFraction = accelRestNorm_ > 1.0f
        ? fabsf(accelNorm / accelRestNorm_ - 1.0f) : 1.0f;
    bool candidate = gyroWindowCount_ == kStationaryWindow &&
                     rateNorm <= AngleConfig::STATIONARY_MAX_RATE_DPS &&
                     accelFraction <= AngleConfig::STATIONARY_ACCEL_TOLERANCE;
    float meanRateSquared = 0.0f;
    for (int axis = 0; axis < 3 && candidate; ++axis) {
        const float n = static_cast<float>(gyroWindowCount_);
        const float mean = gyroWindowSum_[axis] / n;
        meanRateSquared += mean * mean;
        const float variance = fmaxf(0.0f, gyroWindowSumSq_[axis] / n - mean * mean);
        candidate = sqrtf(variance) <= AngleConfig::STATIONARY_MAX_GYRO_STD_DPS;
    }
    candidate = candidate &&
                sqrtf(meanRateSquared) <= AngleConfig::STATIONARY_MAX_MEAN_RATE_DPS;

    if (!candidate) {
        stationaryCandidateSinceMs_ = 0;
        stationary_ = false;
    } else if (stationaryCandidateSinceMs_ == 0) {
        stationaryCandidateSinceMs_ = nowMs;
        stationary_ = false;
    } else {
        stationary_ = static_cast<uint32_t>(nowMs - stationaryCandidateSinceMs_) >=
                      AngleConfig::STATIONARY_HOLD_MS;
    }
    diagnostics_.stationary = stationary_;
}

void AngleReader::updateOnlineBias(float dt) {
    if (!stationary_ || diagnostics_.calibrationState == AngleCalibrationState::Measuring) return;
    const float alpha = dt / (AngleConfig::ONLINE_BIAS_TIME_CONSTANT_S + dt);
    const float maxStep = AngleConfig::ONLINE_BIAS_MAX_SLEW_DPS_PER_S * kDegToRad * dt;
    for (int i = 0; i < 3; ++i) {
        const float requested = alpha * (latestRawGyroRadS_[i] - gyroBiasRadS_[i]);
        gyroBiasRadS_[i] += clampf(requested, -maxStep, maxStep);
        diagnostics_.gyroBiasDps[i] = gyroBiasRadS_[i] * kRadToDeg;
    }
}

void AngleReader::updateMeasurementGates(float accelNorm, float magNorm, bool magFresh) {
    const float accelDeviation = accelRestNorm_ > 1.0f
        ? fabsf(accelNorm / accelRestNorm_ - 1.0f) : 1.0f;
    if (accelValid_) {
        if (!isfinite(accelDeviation) || accelDeviation > AngleConfig::ACCEL_VALID_EXIT_FRACTION) {
            accelValid_ = false;
            accelGoodCount_ = 0;
        }
    } else if (accelDeviation < AngleConfig::ACCEL_VALID_ENTER_FRACTION) {
        if (++accelGoodCount_ >= AngleConfig::ACCEL_VALID_CONFIRM_SAMPLES) {
            accelValid_ = true;
            accelGoodCount_ = AngleConfig::ACCEL_VALID_CONFIRM_SAMPLES;
        }
    } else {
        accelGoodCount_ = 0;
    }

    if (magRestNorm_ <= 0.0f && magFresh &&
        magNorm >= AngleConfig::MAG_NORM_MIN_UT && magNorm <= AngleConfig::MAG_NORM_MAX_UT) {
        magRestNorm_ = magNorm;
    }
    const bool freshRecently = static_cast<uint32_t>(micros() - lastFreshMagUs_) <=
                               AngleConfig::MAG_MAX_STALE_US;
    const float magDeviation = magRestNorm_ > 1.0f
        ? fabsf(magNorm / magRestNorm_ - 1.0f) : 1.0f;
    // An uncalibrated magnetometer can be stable yet point in a completely
    // wrong direction (confirmed on the inherited assembly). Never let it pull
    // the angle toward a false reference. The diagnostic stream remains active
    // so a proper ellipsoid calibration can be collected.
    const bool magPlausible = magCalibration_.verified &&
                              freshRecently && isfinite(magNorm) &&
                              magNorm >= AngleConfig::MAG_NORM_MIN_UT &&
                              magNorm <= AngleConfig::MAG_NORM_MAX_UT;
    if (magValid_) {
        if (!magPlausible || magDeviation > AngleConfig::MAG_VALID_EXIT_FRACTION) {
            magValid_ = false;
            magGoodCount_ = 0;
        }
    } else if (magPlausible && magDeviation < AngleConfig::MAG_VALID_ENTER_FRACTION) {
        if (++magGoodCount_ >= AngleConfig::MAG_VALID_CONFIRM_SAMPLES) {
            magValid_ = true;
            magGoodCount_ = AngleConfig::MAG_VALID_CONFIRM_SAMPLES;
        }
    } else {
        magGoodCount_ = 0;
    }

    diagnostics_.accelValid = accelValid_;
    diagnostics_.magValid = magValid_;
}

void AngleReader::updateTimingStats(float dt) {
    if (dtCount_ == 0) {
        diagnostics_.dtMin = diagnostics_.dtMax = dt;
    } else {
        diagnostics_.dtMin = fminf(diagnostics_.dtMin, dt);
        diagnostics_.dtMax = fmaxf(diagnostics_.dtMax, dt);
    }
    dtSum_ += dt;
    ++dtCount_;
    diagnostics_.dtMean = static_cast<float>(dtSum_ / static_cast<double>(dtCount_));
}

float AngleReader::sampleAndCalculateAngle() {
    const uint32_t nowUs = micros();
    if (!hwOk_) return lastAngleDeg_;

    if (!initialized_) {
        Vec3 gyro, accel, mag;
        if (!readSample(gyro, accel, mag)) return lastAngleDeg_;
        accelRestNorm_ = diagnostics_.calibrationState == AngleCalibrationState::Valid
            ? accelRestNorm_ : norm3(accel);
        updateMeasurementGates(norm3(accel), norm3(mag), diagnostics_.magFresh);
        ekf_.setReferences(q_, accel, mag);
        lastUpdateUs_ = nowUs;
        initialized_ = true;
        return lastAngleDeg_;
    }

    const uint32_t elapsedUs = nowUs - lastUpdateUs_;
    if (elapsedUs < AngleConfig::SAMPLE_INTERVAL_US / 2) return lastAngleDeg_;

    Vec3 gyro, accel, mag;
    if (!readSample(gyro, accel, mag)) return lastAngleDeg_;
    lastUpdateUs_ = nowUs;
    const float dt = elapsedUs * 1.0e-6f;
    diagnostics_.timestampUs = nowUs;
    diagnostics_.dt = dt;
    ++diagnostics_.sequence;

    if (!isfinite(dt) || dt < AngleConfig::MIN_VALID_DT_S || dt > AngleConfig::MAX_VALID_DT_S) {
        ++diagnostics_.invalidDtCount;
        return lastAngleDeg_; // never integrate one current sample over a long gap
    }
    updateTimingStats(dt);
    updateStationarity(gyro, norm3(accel), millis());
    updateOnlineBias(dt);
    updateMeasurementGates(norm3(accel), norm3(mag), diagnostics_.magFresh);

    q_ = ekf_.update(q_, gyro, accel, mag, dt, accelValid_, magValid_);
    EulerDeg e = quatToEulerDeg(q_);
    if (!isfinite(e.roll) || !isfinite(e.pitch) || !isfinite(e.yaw)) return lastAngleDeg_;

    diagnostics_.quaternion[0] = q_.m[0][0];
    diagnostics_.quaternion[1] = q_.m[1][0];
    diagnostics_.quaternion[2] = q_.m[2][0];
    diagnostics_.quaternion[3] = q_.m[3][0];
    diagnostics_.yawDeg = e.yaw;
    diagnostics_.pitchDeg = e.pitch;
    diagnostics_.rollDeg = e.roll;

    // Preserved physical choice: sensor X -> EKF X -> roll. outputZeroDeg_
    // separates mechanical zeroing from the internal orientation state.
    float output = e.roll - outputZeroDeg_;
    while (output > 180.0f) output -= 360.0f;
    while (output <= -180.0f) output += 360.0f;
    lastAngleDeg_ = output;
    diagnostics_.outputAngleDeg = output;
    return lastAngleDeg_;
}

void AngleReader::zeroOutputAngle() {
    const EulerDeg e = quatToEulerDeg(q_);
    if (isfinite(e.roll)) outputZeroDeg_ = e.roll;
}
