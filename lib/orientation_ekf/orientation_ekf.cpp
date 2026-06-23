// ─────────────────────────────────────────────────────────────────────────────
// orientation_ekf.cpp — Implementierung des Quaternion-EKF (Port aus ekf_update.m)
// ─────────────────────────────────────────────────────────────────────────────
#include "orientation_ekf.h"

// ── lokale Hilfsfunktionen (entsprechen den MATLAB-Subfunktionen) ─────────────
namespace {

// normalize_q
Quat normalizeQ(const Quat& q) {
    float n = norm4(q);
    if (n < 1e-12f) { Quat r; r.m[0][0] = 1.0f; return r; }   // Fallback: Einheits-Quat
    return q * (1.0f / n);
}

// omega_mat(w): 4x4 Quaternion-Kinematik-Matrix → q_dot = 0.5*Omega(w)*q (Quaternion-Kinematikmatrix aus der Gyro-Drehzahl w berechnen)
Mat<4, 4> omegaMat(const Vec3& w) {
    float wx = w.m[0][0], wy = w.m[1][0], wz = w.m[2][0];
    Mat<4, 4> M;
    M.m[0][0] = 0;   M.m[0][1] = -wx; M.m[0][2] = -wy; M.m[0][3] = -wz;
    M.m[1][0] = wx;  M.m[1][1] = 0;   M.m[1][2] = wz;  M.m[1][3] = -wy;
    M.m[2][0] = wy;  M.m[2][1] = -wz; M.m[2][2] = 0;   M.m[2][3] = wx;
    M.m[3][0] = wz;  M.m[3][1] = wy;  M.m[3][2] = -wx; M.m[3][3] = 0;
    return M;
}

// W_mat(q): bildet das Gyro-Rauschen (3D) in den Quaternion-Raum (4D) ab.
Mat<4, 3> wMat(const Quat& q) {
    float q1 = q.m[0][0], q2 = q.m[1][0], q3 = q.m[2][0], q4 = q.m[3][0];
    Mat<4, 3> W;
    W.m[0][0] = -q2; W.m[0][1] = -q3; W.m[0][2] = -q4;
    W.m[1][0] =  q1; W.m[1][1] = -q4; W.m[1][2] =  q3;
    W.m[2][0] =  q4; W.m[2][1] =  q1; W.m[2][2] = -q2;
    W.m[3][0] = -q3; W.m[3][1] =  q2; W.m[3][2] =  q1;
    return W * 0.5f;
}

// rot_vec(q, ref): rotiert einen Welt-Referenzvektor in den Body-Frame (Welt→Body).
// Achtung: das ist die transponierte Standard-Rotationsmatrix – exakt wie in MATLAB.
Vec3 rotVec(const Quat& q, const Vec3& ref) {
    float qw = q.m[0][0], qx = q.m[1][0], qy = q.m[2][0], qz = q.m[3][0];
    Mat<3, 3> R;
    R.m[0][0] = 1 - 2*(qy*qy + qz*qz); R.m[0][1] = 2*(qx*qy + qw*qz);     R.m[0][2] = 2*(qx*qz - qw*qy);
    R.m[1][0] = 2*(qx*qy - qw*qz);     R.m[1][1] = 1 - 2*(qx*qx + qz*qz); R.m[1][2] = 2*(qy*qz + qw*qx);
    R.m[2][0] = 2*(qx*qz + qw*qy);     R.m[2][1] = 2*(qy*qz - qw*qx);     R.m[2][2] = 1 - 2*(qx*qx + qy*qy);
    return R * ref;
}

// obs_jacobian(q, ref): 3x4 Jacobi-Matrix von rot_vec nach q (linearisiertes Messmodell).
Mat<3, 4> obsJacobian(const Quat& q, const Vec3& ref) {
    float qw = q.m[0][0], qx = q.m[1][0], qy = q.m[2][0], qz = q.m[3][0];
    float ax = ref.m[0][0], ay = ref.m[1][0], az = ref.m[2][0];
    Mat<3, 4> H;
    H.m[0][0] = ax*qw + ay*qz - az*qy; H.m[0][1] = ax*qx + ay*qy + az*qz;
    H.m[0][2] = -ax*qy + ay*qx - az*qw; H.m[0][3] = -ax*qz + ay*qw + az*qx;

    H.m[1][0] = -ax*qz + ay*qw + az*qx; H.m[1][1] = ax*qy - ay*qx + az*qw;
    H.m[1][2] = ax*qx + ay*qy + az*qz;  H.m[1][3] = -ax*qw - ay*qz + az*qy;

    H.m[2][0] = ax*qy - ay*qx + az*qw;  H.m[2][1] = ax*qz - ay*qw - az*qx;
    H.m[2][2] = ax*qw + ay*qz - az*qy;  H.m[2][3] = ax*qx + ay*qy + az*qz;
    return H * 2.0f;
}

bool allFinite(const Mat<4, 4>& P) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (!isfinite(P.m[i][j])) return false;
    return true;
}

} // namespace

// ── Klassen-Implementierung ───────────────────────────────────────────────────

OrientationEKF::OrientationEKF(float gyroNoise, float accelNoise, float magNoise)
    : gyroNoise_(gyroNoise), accelNoise_(accelNoise), magNoise_(magNoise) {
    P_ = eye<4>();
    // sinnvolle Default-Referenzen aus ekf_update.m (besser: setReferences() aufrufen)
    accelRef_ = vec3(-1.0f, -1.0f, 1.0f);
    float n = norm3(accelRef_);
    accelRef_ = accelRef_ * (1.0f / n);     // im Gegensatz zu MATLAB hier normiert!
    magRef_   = vec3(0.44f, 0.03f, 0.90f);
}

void OrientationEKF::setReferences(const Vec3& accelRef, const Vec3& magRef) {
    float na = norm3(accelRef); accelRef_ = (na > 1e-9f) ? accelRef * (1.0f/na) : accelRef;
    float nm = norm3(magRef);   magRef_   = (nm > 1e-9f) ? magRef   * (1.0f/nm) : magRef;
}

void OrientationEKF::resetCovariance() { P_ = eye<4>(); }

Quat OrientationEKF::update(const Quat& q, const Vec3& gyro,
                            const Vec3& accel, const Vec3& mag, float dt) {
    if (!allFinite(P_)) P_ = eye<4>();   // wie der isempty/isfinite-Check in MATLAB

    // Messungen normieren (Richtung zählt, nicht der Betrag)
    float na = norm3(accel); Vec3 accel_n = (na > 1e-9f) ? accel * (1.0f/na) : accel;
    float nm = norm3(mag);   Vec3 mag_n   = (nm > 1e-9f) ? mag   * (1.0f/nm) : mag;

    // ── Prädiktion ────────────────────────────────────────────────────────────
    Mat<4, 4> Omega  = omegaMat(gyro);
    Quat      q_pred = normalizeQ(q + (Omega * q) * (0.5f * dt));
    Mat<4, 4> F      = eye<4>() + Omega * (0.5f * dt);
    Mat<4, 3> W      = wMat(q);
    Mat<4, 4> P_pred = F * P_ * transpose(F) + (W * transpose(W)) * gyroNoise_;

    // ── Messmodell (6x1): erwartete vs. gemessene Accel-/Mag-Richtung ──────────
    Vec3 h_a = rotVec(q_pred, accelRef_);
    Vec3 h_m = rotVec(q_pred, magRef_);
    Vec6 z, h;
    for (int i = 0; i < 3; ++i) {
        z.m[i][0]     = accel_n.m[i][0]; z.m[i + 3][0] = mag_n.m[i][0];
        h.m[i][0]     = h_a.m[i][0];     h.m[i + 3][0] = h_m.m[i][0];
    }

    // Mess-Jacobi H (6x4)
    Mat<3, 4> Ha = obsJacobian(q_pred, accelRef_);
    Mat<3, 4> Hm = obsJacobian(q_pred, magRef_);
    Mat<6, 4> H;
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 3; ++i) { H.m[i][j] = Ha.m[i][j]; H.m[i + 3][j] = Hm.m[i][j]; }

    // Messrauschen R (6x6, diagonal)
    Mat<6, 6> R;
    R.m[0][0] = R.m[1][1] = R.m[2][2] = accelNoise_;
    R.m[3][3] = R.m[4][4] = R.m[5][5] = magNoise_;

    // ── Korrektur ──────────────────────────────────────────────────────────────
    Mat<6, 6> S = H * P_pred * transpose(H) + R;
    Mat<6, 6> Sinv;
    if (!inverse<6>(S, Sinv)) {
        // S singulär → Korrektur überspringen, nur Prädiktion behalten (robust)
        P_ = P_pred;
        return q_pred;
    }
    Mat<4, 6> K = P_pred * transpose(H) * Sinv;   // Kalman-Gain (4x6)

    P_          = (eye<4>() - K * H) * P_pred;
    Quat q_new  = normalizeQ(q_pred + K * (z - h));
    return q_new;
}

// ── Quaternion → Euler (Grad), ZYX ────────────────────────────────────────────
EulerDeg quatToEulerDeg(const Quat& q) {
    float w = q.m[0][0], x = q.m[1][0], y = q.m[2][0], z = q.m[3][0];
    const float RAD2DEG = 57.2957795f;

    float yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    float sinp  = 2.0f*(w*y - z*x);
    float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(1.5707963f, sinp) : asinf(sinp);
    float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    EulerDeg e;
    e.yaw   = yaw   * RAD2DEG;
    e.pitch = pitch * RAD2DEG;
    e.roll  = roll  * RAD2DEG;
    return e;
}