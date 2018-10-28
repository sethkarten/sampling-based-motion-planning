//
// Created by Seth Karten on 10/26/18.
//
#ifndef SAMPLING_QUATERNION_H
#define SAMPLING_QUATERNION_H
#include <PQP.h>
#include <unordered_set>
class Quaternion {
public:
    double W, X, Y, Z;
    Quaternion(double w, double x, double y, double z);
    Quaternion();
    double distance(const Quaternion& b) const;
    Quaternion interpolate(const Quaternion& b,double f) const;
    PQP_REAL** to_rotation_matrix();
    bool operator==(const Quaternion& q) const;
    static Quaternion uniform_sample(int max);
private:
    double operator*(const Quaternion& b)const;
    Quaternion normalize() const;
    double magnitude()const;
};
#endif //SAMPLING_QUATERNION_H