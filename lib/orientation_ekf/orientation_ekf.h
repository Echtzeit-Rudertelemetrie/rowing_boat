#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// orientation_ekf.h — Quaternion-EKF, portiert aus ekf_update.m
// -> Was macht der ekf (extended Kalman Filter)
//
// Zustand x = q = [w, x, y, z]  (Orientierungs-Quaternion)
//   * Prädiktion:  Gyro integriert die Quaternion vorwärts
//   * Korrektur:   Accelerometer (Schwerkraft) + Magnetometer (Nordrichtung)
//                  ziehen die Schätzung gegen die absoluten Referenzvektoren,
//                  damit der Gyro-Drift nicht aufläuft.
//
// Die Mess-/Prozessrauschwerte und die Referenzvektoren entstammen den
// MATLAB-Kalibrierskripten (sensor_noise_calibration.m, magcal, erste Messung).
// ─────────────────────────────────────────────────────────────────────────────
#include "linalg.h"

class OrientationEKF {
public:
    // Rauschwerte als Varianzen. Defaults = "aktive" Werte aus ekf_update.m. //TODO: Passen die Default werte?
    explicit OrientationEKF(float gyroNoise  = 0.09f,
                            float accelNoise = 0.25f,
                            float magNoise   = 0.64f);

    // Rauschwerte nachträglich setzen (z. B. nach Boot-Kalibrierung im Stillstand).
    void setNoise(float gyroNoise, float accelNoise, float magNoise);

    // Referenzvektoren aus rohen Body-Frame-Messungen setzen (normiert).
    // accelBody/magBody sind unrotierte Sensormessungen; q ist die aktuelle
    // Orientierung -> wird gebraucht, um Body->Welt zurueckzurotieren (Inverse
    // einer Einheits-Quaternion-Rotation = ihre Konjugierte). Empfehlung:
    // einmalig aus der ersten ruhenden Messung übernehmen (accel =
    // Schwerkraftrichtung, mag = lokales Erdfeld).
    void setReferences(const Quat& q, const Vec3& accelBody, const Vec3& magBody);

    // Ein EKF-Schritt. Erwartet:
    //   q     – aktuelle Quaternion (vorheriger Schritt)
    //   gyro  – Drehrate [rad/s]  (Adafruit MPU6050 liefert rad/s)
    //   accel – Beschleunigung [beliebige Einheit, wird normiert]
    //   mag   – Magnetfeld [beliebige Einheit, wird normiert; kalibriert!]
    //   dt    – Zeitschritt [s]
    // Liefert die neue, normierte Quaternion.
    Quat update(const Quat& q, const Vec3& gyro,
                const Vec3& accel, const Vec3& mag, float dt);

    // Kovarianz auf Identität zurücksetzen (z. B. nach einem Reset).
    void resetCovariance();

private:
    Mat<4, 4> P_;          // Fehlerkovarianz
    float     gyroNoise_;  // Prozessrauschen
    float     accelNoise_; // Messrauschen Accel
    float     magNoise_;   // Messrauschen Mag
    Vec3      accelRef_;    // Referenz Schwerkraft (Welt)
    Vec3      magRef_;      // Referenz Magnetfeld  (Welt)
};

// ── Quaternion → Euler-Winkel (Grad), Reihenfolge ZYX = [Yaw, Pitch, Roll] ────
// Entspricht MATLAB eulerd(q,'ZYX','frame'). Für die Ruderblatt-Analyse ist
// laut README der Yaw (Rotation um Z) der relevante Winkel.
struct EulerDeg { float yaw, pitch, roll; };
EulerDeg quatToEulerDeg(const Quat& q);