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
    // Rauschwerte als Varianzen, isotrop (alle drei Achsen gleich). Die Defaults
    // sind erfahrungsbasierte Startwerte, KEINE gemessenen Groessen — gemessene
    // liefert sensor_noise_calibration.m bzw. AngleReader::calibrateSensorNoise().
    explicit OrientationEKF(float gyroNoise  = 0.09f,
                            float accelNoise = 0.25f,
                            float magNoise   = 0.64f);

    // Isotrope Rauschwerte setzen (fuellt alle drei Achsen mit demselben Wert).
    void setNoise(float gyroNoise, float accelNoise, float magNoise);

    // Rauschwerte pro Achse setzen — der Regelfall nach einer Messung.
    // Die Annahme gleicher Achsvarianzen (sigma_wx = sigma_wy = sigma_wz) trifft
    // in der Realitaet praktisch nie zu; liegen die Einzelvarianzen vor, gehoert
    // das Prozessrauschen als W*Sigma*W' gerechnet (statt (W*W')*sigma^2) und R
    // als echte Diagonale. Genau das macht update(), wenn hier Vec3 ankommen.
    // Erwartete Frames/Skalen (wie sensor_noise_calibration.m sie ausgibt):
    //   gyroVar  – Varianz des ROHEN Gyro [(rad/s)^2], EKF-Achsen
    //   accelVar – Varianz des NORMIERTEN Accel-Vektors, dimensionslos
    //   magVar   – Varianz des NORMIERTEN Mag-Vektors, dimensionslos
    void setNoise(const Vec3& gyroVar, const Vec3& accelVar, const Vec3& magVar);

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
    Vec3      gyroNoise_;  // Prozessrauschen, Varianz je EKF-Achse -> Sigma_omega
    Vec3      accelNoise_; // Messrauschen Accel, Varianz je EKF-Achse -> R[0..2]
    Vec3      magNoise_;   // Messrauschen Mag,   Varianz je EKF-Achse -> R[3..5]
    Vec3      accelRef_;    // Referenz Schwerkraft (Welt)
    Vec3      magRef_;      // Referenz Magnetfeld  (Welt)
};

// ── Quaternion → Euler-Winkel (Grad), Reihenfolge ZYX = [Yaw, Pitch, Roll] ────
// Entspricht MATLAB eulerd(q,'ZYX','frame'). Für die Ruderblatt-Analyse ist
// laut README der Yaw (Rotation um Z) der relevante Winkel.
struct EulerDeg { float yaw, pitch, roll; };
EulerDeg quatToEulerDeg(const Quat& q);