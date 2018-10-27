//
// Created by Seth Karten on 10/26/18.
//

#ifndef SAMPLING_QUATERNION_H
#define SAMPLING_QUATERNION_H

class Quaternion {
public:
    double W, X, Y, Z;

    Quaternion(double w, double x, double y, double z);

    double distance(Quaternion a, Quaternion b);

    bool interpolate(Quaternion a, Quaternion b);

    PQP_REAL to_rotation_matrix(Quaternion a);

    Quaternion uniform_sample(int max);

private:

    double operator*(Quaternion a, Quaternion b);

    Quaternion normalize(Quaternion a);

    double magnitude(Quaternion a);
}

#endif //SAMPLING_QUATERNION_H
