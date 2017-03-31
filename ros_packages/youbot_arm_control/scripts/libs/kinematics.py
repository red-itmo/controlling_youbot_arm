#!/usr/bin/env python
import rospy
from math import cos, sin, atan2, pi, sqrt
from scipy import cross, dot, transpose
import tf.transformations as tt
import numpy as np

from ploter import plotIK


class Kinematics:

    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

    def round_rad(self, ang):
        eps = 0.00001
        for i in range(0, len(ang)):
            for j in range(0, len(ang[i])):
                if -eps < ang[i, j] < eps:
                    ang[i, j] = 0#round((ang[i, j]), 3)
        return ang

    def set_dh_parameters(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

    def get_dh_d(self):
        return self.d

    def get_dh_a(self):
        return self.a

    def get_dh_theta(self):
        return self.theta

    def forward(self, qs, f=0, t=6):
        h = tt.identity_matrix()
        for i in range(f, t):
            rz = tt.rotation_matrix(qs[i] + self.theta[i], (0, 0, 1))
            tz = tt.translation_matrix((0, 0, self.d[i]))
            tx = tt.translation_matrix((self.a[i], 0, 0))
            rx = tt.rotation_matrix(self.alpha[i], (1, 0, 0))
            a = tt.concatenate_matrices(rz, tz, tx, rx)
            h = tt.concatenate_matrices(h, a)
            out = "%f\t%f\t%f\t%f + %f" % (self.a[i], self.alpha[i],
                                           self.d[i], qs[i], self.theta[i])
            rospy.logdebug(out)
        xyz = h[:3, 3]
        qtn = tt.quaternion_from_matrix(h)
        rpy = tt.euler_from_matrix(h[:3, :3], axes='sxyz')
        return xyz, qtn, rpy, h

    def getProjectionH(self, t):
        """
        a method of projection a given general goal orientation
            into the subspace of KUKAYoubot' arm
        """
        # divide T(4x4) matrix of R(3x3) and vector p(3x1)
        px, py, pz = t[:3, 3]
        r = t[:3, :3]
        # divide R(3x3) matrix on components
        xt, yt, zt = r[:3, 0], r[:3, 1], r[:3, 2]
        # normal vector to plane of the manipulator
        pxy = sqrt(px ** 2 + py ** 2)
        m = dot(1 / pxy, [-py, px, 0])
        # normal vector to both zt and m
        k = cross(zt, m)
        # new vector zt
        pzt = cross(m, k)
        # t is angle between zt and plane of manipulator
        cost = dot(zt, pzt)
        sint = dot(cross(pzt, zt), k)
        # Rodrigues' formula
        pyt = dot(cost, yt) + dot(sint, cross(yt, k)) + \
            dot(dot((1 - cost), dot(k, yt)), k)
        pxt = cross(pyt, pzt)
        # new rotate matrix
        pr = transpose([pxt, pyt, pzt])
        t[:3, :3] = pr
        return t

    def getVector2to4Frame(self, q1, pr, p):
        px, py, pz = p
        c1, s1 = cos(q1), sin(q1)
        px24 = px * c1 + py * s1 - self.d[5] * (pr[0, 2] * c1 + pr[1, 2] * s1) - self.a[1]
        py24 = px * s1 - py * c1 - self.d[5] * (pr[0, 2] * s1 - pr[1, 2] * c1)
        # py24 = px * s1 - py * c1 - self.d[5] * pr[0, 2] * s1 + self.d[5] * pr[1, 2] * c1
        pz24 = pz - self.d[0] - self.d[1] - self.d[5] * pr[2, 2]
        return px24, py24, pz24

    def inverse(self, xyz, rpy, t):
        eps = np.finfo(np.float).eps
        qs = np.zeros(20).reshape(4, 5)
        t = self.getProjectionH(t)
        px, py, pz = t[:3, 3]
        pr = t[:3, :3]

        # theta_1
        qs[0][0] = qs[1][0] = atan2(py, px)*0
        qs[2][0] = qs[3][0] = pi + atan2(py, px)

        # theta_3
        # cosine theorem
        px24, py24, pz24 = self.getVector2to4Frame(qs[0][0], pr, [px, py, pz])
        px24 -= eps # for singulars -pi for q2
        numerator_0 = px24**2 + py24**2 + pz24**2 - self.a[2]**2 - self.a[3]**2
        denominator_0 = 2 * self.a[2] * self.a[3]
        # for singulars +-pi/2 and +- pi plusing 100 * eps with sign of numerator
        cosq3_0 = numerator_0 / denominator_0 - np.sign(numerator_0) * 100 * eps
        if cosq3_0 <= 1:
            qs[0][2] = atan2(-sqrt(1.0 - cosq3_0 ** 2), cosq3_0)
            qs[1][2] = atan2(sqrt(1.0 - cosq3_0 ** 2), cosq3_0)
        else:
            print("NO SOLUTIONS-")
            print("cosq3: %.20f" % cosq3_0)

        # theta_2
        beta0 = (atan2(self.a[3] * sin(qs[0][2]), self.a[2] + self.a[3] * cos(qs[0][2])))
        phi_0 = -(np.sign(px24)) * atan2(sqrt(px24 ** 2 + py24 ** 2), pz24)
        qs[0][1] = phi_0 - beta0
        qs[1][1] = phi_0 + beta0
        # theta_4
        s234_0 = pr[0, 2] * cos(qs[0][0]) + pr[1, 2] * sin(qs[0][0])
        c234 = pr[2, 2]
        q234_0 = atan2(s234_0, c234) - pi/2
        qs[0][3] = -q234_0 - qs[0][1] - qs[0][2]
        qs[1][3] = -q234_0 - qs[1][1] - qs[1][2]

        # theta_5
        s5_0 = pr[0, 0] * sin(qs[0][0]) - pr[1, 0] * cos(qs[0][0])
        c5_0 = pr[0, 1] * sin(qs[0][0]) - pr[1, 1] * cos(qs[0][0])
        qs[0][4] = atan2(s5_0, c5_0)
        qs[1][4] = atan2(s5_0, c5_0)
        return qs


if __name__ == '__main__':
    DH_A = (0, 0.033, 0.155, 0.135, 0, 0)
    DH_ALPHA = (0, pi / 2, 0, 0, pi / 2, 0)
    DH_D = (0.128, 0.019, 0, 0, 0, 0.218)   # a0 = 0.096 in standart urdf
    DH_THETA = (0, 0, pi / 2, 0, 0, 0)

    # check ik
    ks = Kinematics(DH_A, DH_ALPHA, DH_D, DH_THETA)

    x, q, r, h = ks.forward([0, 0, 0, 0, 0, 0, 0])

    q = ks.inverse(x, r, h)
    print(q)

    plotIK(ks, q, h)



