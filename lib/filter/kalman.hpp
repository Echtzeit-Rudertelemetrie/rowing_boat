#pragma once

#include <math.h>
#include <stdexcept>
#include <array>
#include <BasicLinearAlgebra.h>

using Vec3f  = BLA::Matrix<3, 1, float>;
using Vec4f  = BLA::Matrix<4, 1, float>;
using Vec6f  = BLA::Matrix<6, 1, float>;   // neu
using Mat3f  = BLA::Matrix<3, 3, float>;
using Mat4f  = BLA::Matrix<4, 4, float>;
using Mat6f  = BLA::Matrix<6, 6, float>;   // neu
using Mat34f = BLA::Matrix<3, 4, float>;
using Mat43f = BLA::Matrix<4, 3, float>;
using Mat46f = BLA::Matrix<4, 6, float>;   // neu
using Mat64f = BLA::Matrix<6, 4, float>;   // neu

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
    Mat6f R;   // war Mat3f — jetzt 6×6

    std::array<float, 3> noises;
    float gyro_noise;
    float accel_noise;
    float magnet_noise;

    static float vec3Norm(const Vec3f &v)
    {
        return sqrtf(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
    }

    static float vec4Norm(const Vec4f &v)
    {
        return sqrtf(v(0)*v(0) + v(1)*v(1) + v(2)*v(2) + v(3)*v(3));
    }

    static Vec3f normalizeVec3(const Vec3f &v)
    {
        Vec3f out = v;
        float n = vec3Norm(out);
        if (n > 0.0f) out = out * (1.0f / n);
        return out;
    }

    static Vec4f normalizeVec4(const Vec4f &v)
    {
        Vec4f out = v;
        float n = vec4Norm(out);
        if (n > 0.0f) out = out * (1.0f / n);
        return out;
    }

    static Mat3f identity3()
    {
        Mat3f I; I.Fill(0.0f);
        I(0,0) = 1.0f; I(1,1) = 1.0f; I(2,2) = 1.0f;
        return I;
    }

    static Mat4f identity4()
    {
        Mat4f I; I.Fill(0.0f);
        I(0,0) = 1.0f; I(1,1) = 1.0f; I(2,2) = 1.0f; I(3,3) = 1.0f;
        return I;
    }

    Mat4f Omega(const Vec3f &v)
    {
        Mat4f m;
        m(0,0) =  0.0f;  m(0,1) = -v(0);  m(0,2) = -v(1);  m(0,3) = -v(2);
        m(1,0) =  v(0);  m(1,1) =  0.0f;  m(1,2) =  v(2);  m(1,3) = -v(1);
        m(2,0) =  v(1);  m(2,1) = -v(2);  m(2,2) =  0.0f;  m(2,3) =  v(0);
        m(3,0) =  v(2);  m(3,1) =  v(1);  m(3,2) = -v(0);  m(3,3) =  0.0f;
        return m;
    }

    // ── R jetzt 6×6 ──────────────────────────────────────────────────────────
    Mat6f setMeasurementNoiseCovariance(
        std::array<float, 3> default_noises = {0.09f, 0.25f, 0.64f})
    {
        noises        = default_noises;
        gyro_noise    = noises[0];
        accel_noise   = noises[1];
        magnet_noise  = noises[2];

        Mat6f out; out.Fill(0.0f);
        out(0,0) = accel_noise;
        out(1,1) = accel_noise;
        out(2,2) = accel_noise;
        out(3,3) = magnet_noise;
        out(4,4) = magnet_noise;
        out(5,5) = magnet_noise;
        return out;
    }

    Vec4f predictQuaternion(const Vec4f &quaternion, const Vec3f &omega, float dt)
    {
        Mat4f A = identity4() + (Omega(omega) * (0.5f * dt));
        return normalizeVec4(A * quaternion);
    }

    Mat4f stateTransitionJacobian(const Vec3f &omega, float dt)
    {
        return identity4() + Omega(omega * (0.5f * dt));
    }

    Mat43f processJacobianW(const Vec4f &q, float dt)
    {
        Mat43f W;
        W(0,0) = -q(1);  W(0,1) = -q(2);  W(0,2) = -q(3);
        W(1,0) =  q(0);  W(1,1) = -q(3);  W(1,2) =  q(2);
        W(2,0) =  q(3);  W(2,1) =  q(0);  W(2,2) = -q(1);
        W(3,0) = -q(2);  W(3,1) =  q(1);  W(3,2) =  q(0);
        return W * (0.5f * dt);
    }

    // ── Generisches h(q, ref) = R^T(q) * ref ─────────────────────────────────
    Vec3f measurementModelForRef(const Vec4f &q, const Vec3f &ref)
    {
        const float qw = q(0), qx = q(1), qy = q(2), qz = q(3);
        const float rx = ref(0), ry = ref(1), rz = ref(2);

        Mat3f Rt;
        Rt(0,0) = 1.0f - 2.0f*(qy*qy + qz*qz);
        Rt(0,1) = 2.0f*(qx*qy + qw*qz);
        Rt(0,2) = 2.0f*(qx*qz - qw*qy);
        Rt(1,0) = 2.0f*(qx*qy - qw*qz);
        Rt(1,1) = 1.0f - 2.0f*(qx*qx + qz*qz);
        Rt(1,2) = 2.0f*(qy*qz + qw*qx);
        Rt(2,0) = 2.0f*(qx*qz + qw*qy);
        Rt(2,1) = 2.0f*(qy*qz - qw*qx);
        Rt(2,2) = 1.0f - 2.0f*(qx*qx + qy*qy);

        Vec3f r; r(0) = rx; r(1) = ry; r(2) = rz;
        return Rt * r;
    }

    // ── Kombiniertes 6D-Messmodell [accel; mag] ───────────────────────────────
    Vec6f measurementModel(const Vec4f &q)
    {
        Vec3f a = measurementModelForRef(q, accel_ref);
        Vec3f m = measurementModelForRef(q, magnet_ref);
        Vec6f out;
        out(0) = a(0); out(1) = a(1); out(2) = a(2);
        out(3) = m(0); out(4) = m(1); out(5) = m(2);
        return out;
    }

    // ── Generische 3×4 Jacobian für beliebigen Referenzvektor ─────────────────
    Mat34f observationJacobianForRef(const Vec4f &q, const Vec3f &ref)
    {
        const float qw = q(0), qx = q(1), qy = q(2), qz = q(3);
        const float ax = ref(0), ay = ref(1), az = ref(2);

        Mat34f H;
        H(0,0) =  ax*qw + ay*qz - az*qy;
        H(0,1) =  ax*qx + ay*qy + az*qz;
        H(0,2) = -ax*qy + ay*qx - az*qw;
        H(0,3) = -ax*qz + ay*qw + az*qx;

        H(1,0) = -ax*qz + ay*qw + az*qx;
        H(1,1) =  ax*qy - ay*qx + az*qw;
        H(1,2) =  ax*qx + ay*qy + az*qz;
        H(1,3) = -ax*qw - ay*qz + az*qy;

        H(2,0) =  ax*qy - ay*qx + az*qw;
        H(2,1) =  ax*qz - ay*qw - az*qx;
        H(2,2) =  ax*qw + ay*qz - az*qy;
        H(2,3) =  ax*qx + ay*qy + az*qz;

        return H * 2.0f;
    }

    // ── Kombinierte 6×4 Jacobian [H_accel; H_mag] ────────────────────────────
    Mat64f observationJacobian(const Vec4f &q)
    {
        Mat34f Ha = observationJacobianForRef(q, accel_ref);
        Mat34f Hm = observationJacobianForRef(q, magnet_ref);

        Mat64f H;
        for (int j = 0; j < 4; j++)
        {
            H(0,j) = Ha(0,j); H(1,j) = Ha(1,j); H(2,j) = Ha(2,j);
            H(3,j) = Hm(0,j); H(4,j) = Hm(1,j); H(5,j) = Hm(2,j);
        }
        return H;
    }

public:
    EKF(bool useMag = true, float freq = 100.0f)
        : hasMag(useMag), frequency(freq), Dt(1.0f / freq)
    {
        P = identity4();
        R = setMeasurementNoiseCovariance({0.09f, 0.25f, 0.64f});

        accel_ref(0) = 0.0f; accel_ref(1) = 0.0f; accel_ref(2) = 1.0f;
        magnet_ref(0) = 1.0f; magnet_ref(1) = 0.0f; magnet_ref(2) = 0.0f;
    }

    // magnet_ref aus erster Messung setzen (optional, aber empfohlen)
    void setMagnetReference(const Vec3f &mag_world)
    {
        magnet_ref = normalizeVec3(mag_world);
    }

    Vec4f update(
        const Vec4f &state_orientation,
        const Vec3f &gyro,
        const Vec3f &accel,
        const Vec3f &magneto,
        float dt = 0.0f)
    {
        const float timestep = (dt == 0.0f) ? this->Dt : dt;

        const float accel_norm = vec3Norm(accel);
        if (accel_norm == 0.0f) return state_orientation;

        const float mag_norm = vec3Norm(magneto);
        if (mag_norm == 0.0f) return state_orientation;

        // ── Predict ───────────────────────────────────────────────────────────
        Vec4f predictedState = predictQuaternion(state_orientation, gyro, timestep);

        Mat4f F  = stateTransitionJacobian(gyro, timestep);
        Mat43f W = processJacobianW(state_orientation, timestep);
        Mat4f predictedCovariance = F * this->P * ~F + (W * ~W) * gyro_noise;

        // ── Measurement vector [accel_norm; mag_norm] ─────────────────────────
        Vec3f a_n = accel   * (1.0f / accel_norm);
        Vec3f m_n = magneto * (1.0f / mag_norm);

        Vec6f measurementVector;
        measurementVector(0) = a_n(0); measurementVector(1) = a_n(1); measurementVector(2) = a_n(2);
        measurementVector(3) = m_n(0); measurementVector(4) = m_n(1); measurementVector(5) = m_n(2);

        // ── Update ────────────────────────────────────────────────────────────
        Vec6f expectedMeasurement = measurementModel(predictedState);
        Vec6f innovation          = measurementVector - expectedMeasurement;

        Mat64f H   = observationJacobian(predictedState);   // 6×4
        Mat46f PHt = predictedCovariance * ~H;              // 4×6
        Mat6f  S   = H * PHt + this->R;                     // 6×6
        Mat46f K   = PHt * BLA::Inverse(S);                 // 4×6

        this->P = (identity4() - K * H) * predictedCovariance;

        return normalizeVec4(predictedState + K * innovation);
    }
};
