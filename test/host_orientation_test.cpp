#include <cassert>
#include <cmath>
#include <iostream>
#include "orientation_ekf.h"

static Quat identity() {
    Quat q;
    q.m[0][0] = 1.0f;
    return q;
}

static bool finiteQuat(const Quat& q) {
    for (int i = 0; i < 4; ++i)
        if (!std::isfinite(q.m[i][0])) return false;
    return true;
}

int main() {
    OrientationEKF ekf(1.0e-4f, 2.5e-3f, 5.0e-3f);
    Quat q = identity();
    const Vec3 gravity = vec3(1.0f, 0.0f, 0.0f);
    const Vec3 magnetic = vec3(0.2f, 0.8f, 0.1f);
    ekf.setReferences(q, gravity, magnetic);

    for (int i = 0; i < 2000; ++i) {
        q = ekf.update(q, vec3(0.001f, -0.001f, 0.002f),
                       gravity, magnetic, 0.01f, true, true);
        assert(finiteQuat(q));
        assert(std::fabs(norm4(q) - 1.0f) < 1.0e-4f);
    }

    // Rejected zero measurement vectors and invalid input must not create NaN.
    q = ekf.update(q, vec3(0.0f, 0.0f, 0.0f),
                   vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f),
                   0.01f, false, false);
    assert(finiteQuat(q));
    Quat recovered = ekf.update(q, vec3(NAN, 0.0f, 0.0f),
                               gravity, magnetic, 0.01f, true, true);
    assert(finiteQuat(recovered));
    std::cout << "host_orientation_test: PASS\n";
}
