from math import sin, cos, fabs, acos, sqrt
from random import random as rand
import sys
sys.path.insert(0, '../../pqp_server/pyscript')
from pqp_ros_client import pqp_client


class SE3:
    def __init__(x,y,z,q):
        self.X = x
        self.Y = y
        self.Z = z
        self.q = q

    '''
    Returns True if there is a collision
    '''
    @staticmethod
    def check_collide(self, other):
        x_res = y_res = z_res = 0.1
        if self.X > other.X:
            x_res *= -1
        if self.Y > other.Y:
            x_res *= -1
        if self.Z > other.Z:
            x_res *= -1
        x = self.X
        y = self.Y
        z = self.Z
        q = self.q
        while fabs(x - other.X) > 0.1\
        and fabs(y - other.Y) > 0.1\
        and fabs(Z - other.Z) > 0.1\
        and fabs(Quaternion.distance(q, other.q)) > 0.1:
            T = [x,y,z]
            R = q.to_rotation_matrix()
            if pqp_client(T, R):
                # Collision
                return True
            x += x_res
            y += y_res
            z += z_res
            q = Quaternion.interpolate(q, other.q)
        return False


class Quaternion:
    def __init__(w, x, y, z):
            self.W = w
            self.X = x
            self.Y = y
            self.Z = z

    def __mul__(self, other):
        return self.W * other.W + self.X * other.X + self.Y * other.Y + self.Z * other.Z

    def __rmul__(self, other):
        return self.W * other.W + self.X * other.X + self.Y * other.Y + self.Z * other.Z

    @staticmethod
    def distance(a, b):
        l = a.normalize() * other.normalize()
        return 1 - fabs(l)

    @staticmethod
    def interpolate(a, b, res=0.1):
        # slerp (spherical linear interpolation)
        f = res    # step [0,1]
        # Quaternion inner product
        l = a.normalize() * b.normalize()
        if(l < 0):
            # the quaternions are pointing in opposite directions so,
            # use the equivalent alternative representation of b
            other.W *= -1
            other.X *= -1
            other.Y *= -1
            other.Z *= -1
            l *= -1

        # Calculate spherical linear interpolation factors
        alpha = acos(l)
        gamma = 1 / sin(alpha)
        r = sin((1 - f) * alpha) * gamma
        s = sin(f * alpha) * gamma
        # set the interpolated quaternion
        w = r * a.W + s * b.W
        x = r * a.X + s * b.X
        y = r * a.Y + s * b.Y
        z = r * a.Z + s * b.Z
        Q = Quaternion( w, x, y, z)
        return Q.normalize()

    def to_rotation_matrix(self):
        r_m = [0]*9     # rotation matrix

        r_m[0] = self.W * self.W + self.X * self.X - self.Y * self.Y - self.Z * self.Z
        r_m[1] = 2 * (self.X * self.Y - self.W * self.Z)
        r_m[2] = 2 * (self.W * self.Y + self.X * self.Z)
        r_m[3] = 2 * (self.X * self.Y + self.W * self.Z)
        r_m[4] = self.W * self.W - self.X * self.X + self.Y * self.Y - self.Z * self.Z
        r_m[5] = 2 * (self.Y * self.Z - self.W * self.X)
        r_m[6] = 2 * (self.X * self.Z - self.W * self.Y)
        r_m[7] = 2 * (self.W * self.X - self.Y * self.Z)
        r_m[8] = self.W * self.W - self.X * self.X - self.Y * self.Y + self.Z * self.Z

        return r_m

    @staticmethod
    def uniform_sample(max):
        s = rand()*max
        sigma1 = sqrt(1 - s)
        sigma2 = sqrt(s)
        theta1 = 2 * M_PI * rand()*max
        theta2 = 2 * M_PI * rand()*max
        w = cos(theta2) * sigma2
        x = sin(theta1) * sigma1
        y = cos(theta1) * sigma1
        z = sin(theta2) * sigma2
        return Quaternion(w, x, y, z)  # returns normalized quaternions

    def normalize(self):
        mag = sqrt(self.magnitude())   # Magnitude
        return Quaternion(self.W / mag, self.X / mag, self.Y / mag, self.Z / mag)

    def magnitude(self):
        return self.W * self.W + self.X * self.X + self.Y * self.Y + self.Z * self.Z
