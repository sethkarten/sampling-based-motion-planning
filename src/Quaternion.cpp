//
// Created by Seth Karten on 10/26/18.
//
#include <Quaternion.hpp>

using namespace std;
namespace std {
    template<>
    class hash<class Quaternion> {
    public:
        size_t operator()(const Quaternion &q) const{
            return std::hash<double>()(q.W + q.X + q.Y + q.Z);
        }
    };
};
Quaternion::Quaternion(double w, double x, double y, double z) {
    W = w;
    X = x;
    Y = y;
    Z = z;
}
Quaternion::Quaternion() {
    W = 0;
    X = 0;
    Y = 0;
    Z = 0;
}
double Quaternion::distance(const Quaternion& b) const {
    double lambda = normalize() * b.normalize();
    return 1 - fabs(lambda);
}
bool Quaternion::operator==(const Quaternion &q) const {
    return distance(q) <= .00001;
}
Quaternion Quaternion::interpolate(const Quaternion& b,double f) const{
    // slerp (spherical linear interpolation)
    // Quaternion inner product
    Quaternion tmp = b.normalize();
    double lambda = normalize() * tmp;
    if(lambda < 0) {
        // the quaternions are pointing in opposite directions so,
        // use the equivalent alternative representation of b
        tmp.W *= -1;
        tmp.X *= -1;
        tmp.Y *= -1;
        tmp.Z *= -1;
        lambda *= -1;
    }
    // Calculate spherical linear interpolation factors
    double alpha = acos(lambda);
    double gamma = 1 / sin(alpha);
    double r = sin((1 - f) * alpha) * gamma;
    double s = sin(f * alpha) * gamma;
    // set the interpolated quaternion
    double w = r * W + s * tmp.W;
    double x = r * X + s * tmp.X;
    double y = r * Y + s * tmp.Y;
    double z = r * Z + s * tmp.Z;
    Quaternion Q( w, x, y, z);
    return Q.normalize();
}
PQP_REAL** Quaternion::to_rotation_matrix() {
    PQP_REAL** r_m = (PQP_REAL**) malloc(sizeof(PQP_REAL*)*3);     // rotation matrixo
    for (int i = 0 ; i < 3; i++){
        r_m[i] = (PQP_REAL*) malloc(sizeof(PQP_REAL)*3);
    }
    r_m[0][0] = W * W + X * X - Y * Y - Z * Z;
    r_m[0][1] = 2 * (X * Y - W * Z);
    r_m[0][2] = 2 * (W * Y + X * Z);
    r_m[1][0] = 2 * (X * Y + W * Z);
    r_m[1][1] = W * W - X * X + Y * Y - Z * Z;
    r_m[1][2] = 2 * (Y * Z - W * X);
    r_m[2][0] = 2 * (X * Z - W * Y);
    r_m[2][1] = 2 * (W * X - Y * Z);
    r_m[2][2] = W * W - X * X - Y * Y + Z * Z;
    return r_m;
}
Quaternion Quaternion::uniform_sample(double a, double b, double c) {
    rand();
    double s = a;
    double sigma1 = sqrt(1 - s);
    double sigma2 = sqrt(s);
    double theta1 = 2 * M_PI * b;
    double theta2 = 2 * M_PI * c;
    double w = cos(theta2) * sigma2;
    double x = sin(theta1) * sigma1;
    double y = cos(theta1) * sigma1;
    double z = sin(theta2) * sigma2;
    return Quaternion(w, x, y, z);  // returns normalized quaternions
}
double Quaternion::operator * (const Quaternion& b)const {
    return W * b.W + X * b.X + Y * b.Y + Z * b.Z;
}
Quaternion Quaternion::normalize() const{
    double mag = magnitude();   // Magnitude
    Quaternion tmp;
    tmp.W = W/mag;
    tmp.X = X/mag;
    tmp.Y = Y/mag;
    tmp.Z = Z/mag;
    return tmp;
}
double Quaternion::magnitude() const {
    return sqrt((*this)*(*this));
}
