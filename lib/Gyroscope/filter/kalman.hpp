#pragma once

#include <math.h>
#include <stdexcept>
#include <array>
#include <BasicLinearAlgebra.h>

using Vec3f  = BLA::Matrix<3, 1, float>;
using Vec4f  = BLA::Matrix<4, 1, float>;
using Mat3f  = BLA::Matrix<3, 3, float>;
using Mat4f  = BLA::Matrix<4, 4, float>;
using Mat34f = BLA::Matrix<3, 4, float>;
using Mat43f = BLA::Matrix<4, 3, float>;

using ValueError = std::invalid_argument;

class EKF
{
private:
    bool hasMag;
    Vec3f accel_ref;
    Vec3f magnet_ref;

    float frequency;
    float Dt;

    Mat4f P;
    Mat3f R;

    std::array<float, 3> noises;
    float gyro_noise;
    float accel_noise;
    float magnet_noise;

    static float vec3Norm(const Vec3f &v)
    {
        return sqrtf(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
    }

    static float vec4Norm(const Vec4f &v)
    {
        return sqrtf(v(0) * v(0) + v(1) * v(1) + v(2) * v(2) + v(3) * v(3));
    }

    static Vec3f normalizeVec3(const Vec3f &v)
    {
        Vec3f out = v;
        float n = vec3Norm(out);
        if (n > 0.0f)
        {
            out = out * (1.0f / n);
        }
        return out;
    }

    static Vec4f normalizeVec4(const Vec4f &v)
    {
        Vec4f out = v;
        float n = vec4Norm(out);
        if (n > 0.0f)
        {
            out = out * (1.0f / n);
        }
        return out;
    }

    static Mat3f identity3()
    {
        Mat3f I;
        I.Fill(0.0f);
        I(0, 0) = 1.0f;
        I(1, 1) = 1.0f;
        I(2, 2) = 1.0f;
        return I;
    }

    static Mat4f identity4()
    {
        Mat4f I;
        I.Fill(0.0f);
        I(0, 0) = 1.0f;
        I(1, 1) = 1.0f;
        I(2, 2) = 1.0f;
        I(3, 3) = 1.0f;
        return I;
    }

    Mat3f skew(const Vec3f &v)
    {
        Mat3f S;
        S(0, 0) = 0.0f;   S(0, 1) = -v(2);  S(0, 2) = v(1);
        S(1, 0) = v(2);   S(1, 1) = 0.0f;   S(1, 2) = -v(0);
        S(2, 0) = -v(1);  S(2, 1) = v(0);   S(2, 2) = 0.0f;
        return S;
    }

    Mat3f setMeasurementNoiseCovariance(
        std::array<float, 3> default_noises = {0.09f, 0.25f, 0.64f})
    {
        noises = default_noises;
        gyro_noise = noises[0];
        accel_noise = noises[1];
        magnet_noise = noises[2];

        Mat3f out;
        out.Fill(0.0f);
        out(0, 0) = accel_noise;
        out(1, 1) = accel_noise;
        out(2, 2) = accel_noise;
        return out;
    }

    Mat4f Omega(const Vec3f &v)
    {
        Mat4f m;
        m(0, 0) = 0.0f;   m(0, 1) = -v(0);  m(0, 2) = -v(1);  m(0, 3) = -v(2);
        m(1, 0) = v(0);   m(1, 1) = 0.0f;   m(1, 2) = v(2);   m(1, 3) = -v(1);
        m(2, 0) = v(1);   m(2, 1) = -v(2);  m(2, 2) = 0.0f;   m(2, 3) = v(0);
        m(3, 0) = v(2);   m(3, 1) = v(1);   m(3, 2) = -v(0);  m(3, 3) = 0.0f;
        return m;
    }

    Vec4f predictQuaternion(const Vec4f &quaternion, const Vec3f &omega, float dt)
    {
        Mat4f A = identity4() + (Omega(omega) * (0.5f * dt));
        return normalizeVec4(A * quaternion);
    }

    Mat4f stateTransitionJacobian(const Vec3f &omega, float dt)
    {
        Vec3f scaledAngularVelocity = omega * (0.5f * dt);
        return identity4() + Omega(scaledAngularVelocity);
    }

    Vec3f measurementModel(const Vec4f &q)
    {
        const float qw = q(0);
        const float qx = q(1);
        const float qy = q(2);
        const float qz = q(3);

        const float ax = accel_ref(0);
        const float ay = accel_ref(1);
        const float az = accel_ref(2);

        Mat3f Rt;

        Rt(0, 0) = 1.0f - 2.0f * (qy * qy + qz * qz);
        Rt(0, 1) = 2.0f * (qx * qy + qw * qz);
        Rt(0, 2) = 2.0f * (qx * qz - qw * qy);

        Rt(1, 0) = 2.0f * (qx * qy - qw * qz);
        Rt(1, 1) = 1.0f - 2.0f * (qx * qx + qz * qz);
        Rt(1, 2) = 2.0f * (qy * qz + qw * qx);

        Rt(2, 0) = 2.0f * (qx * qz + qw * qy);
        Rt(2, 1) = 2.0f * (qy * qz - qw * qx);
        Rt(2, 2) = 1.0f - 2.0f * (qx * qx + qy * qy);

        Vec3f ref;
        ref(0) = ax;
        ref(1) = ay;
        ref(2) = az;

        return Rt * ref;
    }

    Mat34f observationJacobian(const Vec4f &q)
    {
        const float qw = q(0);
        const float qx = q(1);
        const float qy = q(2);
        const float qz = q(3);

        const float ax = accel_ref(0);
        const float ay = accel_ref(1);
        const float az = accel_ref(2);

        Mat34f H;

        H(0, 0) = ax * qw + ay * qz - az * qy;
        H(0, 1) = ax * qx + ay * qy + az * qz;
        H(0, 2) = -ax * qy + ay * qx - az * qw;
        H(0, 3) = -ax * qz + ay * qw + az * qx;

        H(1, 0) = -ax * qz + ay * qw + az * qx;
        H(1, 1) = ax * qy - ay * qx + az * qw;
        H(1, 2) = ax * qx + ay * qy + az * qz;
        H(1, 3) = -ax * qw - ay * qz + az * qy;

        H(2, 0) = ax * qy - ay * qx + az * qw;
        H(2, 1) = ax * qz - ay * qw - az * qx;
        H(2, 2) = ax * qw + ay * qz - az * qy;
        H(2, 3) = ax * qx + ay * qy + az * qz;

        return H * 2.0f;
    }

    Mat43f processJacobianW(const Vec4f &q, float dt)
    {
        Mat43f W;

        W(0, 0) = -q(1);
        W(0, 1) = -q(2);
        W(0, 2) = -q(3);

        W(1, 0) = q(0);
        W(1, 1) = -q(3);
        W(1, 2) = q(2);

        W(2, 0) = q(3);
        W(2, 1) = q(0);
        W(2, 2) = -q(1);

        W(3, 0) = -q(2);
        W(3, 1) = q(1);
        W(3, 2) = q(0);

        return W * (0.5f * dt);
    }

public:
    EKF(bool useMag = false, float freq = 100.0f)
        : hasMag(useMag), frequency(freq), Dt(1.0f / freq)
    {
        if (hasMag)
        {
            throw ValueError("BLA EKF version is accelerometer-only. Construct with EKF(false, freq).");
        }

        P = identity4();

        noises = {0.09f, 0.25f, 0.64f};
        gyro_noise = noises[0];
        accel_noise = noises[1];
        magnet_noise = noises[2];

        R = setMeasurementNoiseCovariance(noises);

        accel_ref(0) = 0.0f;
        accel_ref(1) = 0.0f;
        accel_ref(2) = 1.0f;

        magnet_ref(0) = 1.0f;
        magnet_ref(1) = 0.0f;
        magnet_ref(2) = 0.0f;
    }

    Vec4f update(
        const Vec4f &state_orientation,
        const Vec3f &gyro,
        const Vec3f &accel,
        const Vec3f &magneto,
        float dt = 0.0f)
    {
        (void)magneto;

        const float timestep = (dt == 0.0f) ? this->Dt : dt;

        const float qNorm = vec4Norm(state_orientation);
        if (fabsf(qNorm - 1.0f) > 1e-3f)
        {
            throw ValueError("A-priori quaternion must have a norm close to 1.");
        }

        const float accel_norm = vec3Norm(accel);
        if (accel_norm == 0.0f)
        {
            return state_orientation;
        }

        Vec3f measurementVector = accel * (1.0f / accel_norm);

        Vec4f predictedState = predictQuaternion(state_orientation, gyro, timestep);

        Mat4f F = stateTransitionJacobian(gyro, timestep);
        Mat43f W = processJacobianW(state_orientation, timestep);

        Mat4f processCovarianceNoise = (W * ~W) * gyro_noise;
        Mat4f predictedCovariance = F * this->P * ~F + processCovarianceNoise;

        Vec3f expectedMeasurement = measurementModel(predictedState);
        Vec3f innovation = measurementVector - expectedMeasurement;

        Mat34f H = observationJacobian(predictedState);

        Mat43f PHt = predictedCovariance * ~H;
        Mat3f innovationCovariance = H * PHt + this->R;

        Mat3f innovationCovarianceInv = BLA::Inverse(innovationCovariance);
        Mat43f kalmanGain = PHt * innovationCovarianceInv;

        this->P = (identity4() - kalmanGain * H) * predictedCovariance;

        Vec4f correctedState = predictedState + kalmanGain * innovation;
        return normalizeVec4(correctedState);
    }
};

