//
// Created by Seth Karten on 10/26/18.
//
#include <Quaternion.hpp>
#include "PQP.h"
#include <cmath>
#include <stdlib.h>
using namespace std;
class Quaternion {
public:
    double W, X, Y, Z;
    Quaternion(double w, double x, double y, double z) {
        W = w;
        X = x;
        Y = y;
        Z = z;
    }
    double distance(Quaternion a, Quaternion b) {
        lambda = normalize(a) * normalize(b);
        return 1 - abs(lambda);
    }
    bool interpolate(Quaternion a, Quaternion b) {
        // slerp (spherical linear interpolation)
        double f = 0.2;     // step [0,1]
        // Quaternion inner product
        lambda = normalize(a) * normalize(b);
        if(lambda < 0) {
            // the quaternions are pointing in opposite directions so,
            // use the equivalent alternative representation of b
            b.W *= -1;
            b.X *= -1;
            b.Y *= -1;
            b.Z *= -1;
            lambda *= -1;
        }
        // Calculate spherical linear interpolation factors
        alpha = acos(lambda);
        gamma = 1 / sin(alpha);
        r = sin((1 - f) * alpha) * gamma;
        s = sin(f * alpha) * gamma;
        // set the interpolated quaternion
        double w = r * a.W + s * b.W;
        double x = r * a.X + s * b.X;
        double y = r * a.Y + s * b.Y;
        double z = r * a.Z + s * b.Z;
        Q = Quaternion( w, x, y, z);
        return normalize(Q);
    }
    PQP_REAL to_rotation_matrix(Quaternion a) {
        PQP_REAL r_m[3][3];     // rotation matrix
    }
    r_m[0][0] = a.W * a.W + a.X * a.X - a.Y * a.Y - a.Z * a.Z;
    r_m[0][1] = 2 * (a.X * a.Y - a.W * a.Z);
    r_m[0][2] = 2 * (a.W * a.Y + a.X * a.Z);
    r_m[1][0] = 2 * (a.X * a.Y + a.W * a.Z);
    r_m[1][1] = a.W * a.W - a.X * a.X + a.Y * a.Y - a.Z * a.Z;
    r_m[1][2] = 2 * (a.Y * a.Z - a.W * a.X);
    r_m[2][0] = 2 * (a.X * a.Z - a.W * a.Y);
    r_m[2][1] = 2 * (a.W * a.X - a.Y * a.Z);
    r_m[2][2] = a.W * a.W - a.X * a.X - a.Y * a.Y + a.Z * a.Z;
    return r_m;
}
    Quaternion uniform_sample(int max) {
    srand(max);
    s = rand();
    sigma1 = sqrt(1 - s);
    sigma2 = sqrt(s);
    theta1 = 2 * M_PI * rand();
    theta2 = 2 * M_PI * rand();
    w = cos(theta2) * sigma2;
    x = sin(theta1) * sigma1;
    y = cos(theta1) * sigma1;
    z = sin(theta2) * sigma2;
    return Quaternion(w, x, y, z);  // returns normalized quaternions
}
private:
double operator * (Quaternion a, Quaternion b) {
    return a.W * b.W + a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}
Quaternion normalize(Quaternion a) {
    double mag = sqrt(magnitude(a));   // Magnitude
    return Quaternion(a.W / mag, a.X / mag, a.Y / mag, a.Z / mag);
}
double magnitude(Quaternion a) {
    return a.W * a.W + a.X * a.X + a.Y * a.Y + a.Z * a.Z;
}
};