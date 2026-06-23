#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// linalg.h — Minimale, statische Matrix-Bibliothek für Embedded (ESP32)
//
// Ziele:
//   * KEINE dynamische Speicherverwaltung (alle Größen sind Compile-Zeit-Konstanten)
//   * float statt double  → der ESP32 hat eine Single-Precision-FPU
//   * nur die Operationen, die das Orientierungs-EKF wirklich braucht
//
// Vektoren werden als Spaltenmatrizen Mat<N,1> dargestellt.
// ─────────────────────────────────────────────────────────────────────────────
#include <math.h>

//template für generische Matrix
template <int R, int C>
struct Mat {
    float m[R][C];

    // Standard-Konstruktor: alles auf 0 (entspricht MATLAB zeros()).
    Mat() {
        for (int i = 0; i < R; ++i)
            for (int j = 0; j < C; ++j)
                m[i][j] = 0.0f;
    }

    //Zugriff per Klammern (float x = A(2, 2); oder A(0, 1) = 3.0f;)
    float&       operator()(int i, int j)       { return m[i][j]; }
    float        operator()(int i, int j) const { return m[i][j]; }
};

// Einheitsmatrix (entspricht MATLAB eye(N)) – nur für quadratische Matrizen.
template <int N>
Mat<N, N> eye() {
    Mat<N, N> I;
    for (int i = 0; i < N; ++i) I.m[i][i] = 1.0f;
    return I;
}

// Matrix Rechenoperationen werden definiert
// Matrix * Matrix, also klassische matrixmanipulation  (innere Dimensionen müssen passen, sonst Compile-Fehler).
template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> out;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) {
            float s = 0.0f;
            for (int k = 0; k < K; ++k) s += a.m[i][k] * b.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

// Matrix + Matrix / Matrix - Matrix (Addition und Subtraktion)
template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> o;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) o.m[i][j] = a.m[i][j] + b.m[i][j];
    return o;
}
template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> o;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) o.m[i][j] = a.m[i][j] - b.m[i][j];
    return o;
}

// Matrix * Skalar
template <int R, int C>
Mat<R, C> operator*(const Mat<R, C>& a, float s) {
    Mat<R, C> o;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) o.m[i][j] = a.m[i][j] * s;
    return o;
}

// Transponieren (entspricht MATLAB A')
template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& a) {
    Mat<C, R> o;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j) o.m[j][i] = a.m[i][j];
    return o;
}

// Inverse quadratischer Matrizen per Gauß-Jordan mit Teil-Pivotisierung.
// Gibt false zurück, falls die Matrix (numerisch) singulär ist.
// 2*N ist Compile-Zeit-konstant → kein VLA, alles auf dem Stack.
template <int N>
bool inverse(const Mat<N, N>& in, Mat<N, N>& out) {
    float a[N][2 * N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            a[i][j]     = in.m[i][j];
            a[i][j + N] = (i == j) ? 1.0f : 0.0f;
        }

    for (int col = 0; col < N; ++col) {
        // Pivot-Zeile mit größtem Betrag suchen (Stabilität)
        int   piv  = col;
        float maxv = fabsf(a[col][col]);
        for (int r = col + 1; r < N; ++r) {
            float v = fabsf(a[r][col]);
            if (v > maxv) { maxv = v; piv = r; }
        }
        if (maxv < 1e-9f) return false;  // singulär -> nicht invertiervar

        if (piv != col)
            for (int j = 0; j < 2 * N; ++j) {
                float t = a[col][j]; a[col][j] = a[piv][j]; a[piv][j] = t;
            }

        float d = a[col][col];
        for (int j = 0; j < 2 * N; ++j) a[col][j] /= d;

        for (int r = 0; r < N; ++r) {
            if (r == col) continue;
            float f = a[r][col];
            if (f != 0.0f)
                for (int j = 0; j < 2 * N; ++j) a[r][j] -= f * a[col][j];
        }
    }

    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) out.m[i][j] = a[i][j + N];
    return true;
}

// ── Bequeme Typ-Aliase und Vektor-Helfer ─────────────────────────────────────
// paar bequemere namen für Vektoren und Quaterione
using Vec3 = Mat<3, 1>;
using Vec6 = Mat<6, 1>;
using Quat = Mat<4, 1>;   // [w, x, y, z]

inline Vec3 vec3(float x, float y, float z) {
    Vec3 v; v.m[0][0] = x; v.m[1][0] = y; v.m[2][0] = z; return v;
}

//normieren der vektoren
inline float norm3(const Vec3& v) {
    return sqrtf(v.m[0][0]*v.m[0][0] + v.m[1][0]*v.m[1][0] + v.m[2][0]*v.m[2][0]);
}
inline float norm4(const Quat& q) {
    return sqrtf(q.m[0][0]*q.m[0][0] + q.m[1][0]*q.m[1][0] +
                 q.m[2][0]*q.m[2][0] + q.m[3][0]*q.m[3][0]);
}