#!/usr/bin/env python
import rospy
import math
import scipy
import tf.transformations as tt

PI = math.pi


class Kinematics:

    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

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
        return xyz, qtn, rpy

    def inverse(self, xyz, rpy):
        qs = []
        px, py, pz = xyz
        r = tt.euler_matrix(rpy[0], rpy[1], rpy[2], axes='sxyz')
        xt = r[:3, 0]
        yt = r[:3, 1]
        zt = r[:3, 2]
        pxy = math.sqrt(px ** 2 + py ** 2)
        # a method of projection a given general goal orientation
        #   into the subspace of KUKAYoubot' arm
        m = scipy.dot(r, [[-py], [px], [0]])
        k = scipy.cross(m, zt)
        pzt = scipy.cross(k, m)
        cost = scipy.dot(zt, pzt)
        sint = scipy.dot(scipy.cross(zt, pzt), k)
        pyt = scipy.dot(cost, yt) + scipy.dot(sint, scipy.cross(k, yt)) + \
            scipy.dot(scipy.dot((1 - cost), scipy.dot(k, yt)), k)
        pxt = scipy.cross(pyt, pzt)
        # kinematic equations
        pr = scipy.transpose([pxt, pyt, pzt])
        q1 = math.atan2(py, px)
        c1 = math.cos(q1)
        s1 = math.sin(q1)
        q5 = math.atan2(pr[1, 0] * c1 - pr[0, 0] * s1,
                        pr[1, 1] * c1 - pr[0, 1] * s1)
        q234 = math.atan2(pr[0, 2] * c1 + pr[1, 2] * s1, pr[2, 2])
        cosq3 = (px ** 2 + py ** 2 + pz ** 2 - self.a[2] ** 2 - self.a[3] ** 2) / \
                (2 * self.a[2] * self.a[3])
        q3 = math.atan2(math.sqrt(1 - cosq3 ** 2), cosq3)
        q2 = -math.atan2(pz, math.sqrt(px ** 2 + py ** 2)) - math.atan2(
            self.a[3] * math.sin(q3), self.a[2] + self.a[3] * math.cos(q3))
        q4 = q234 - q2 - q3
        return q1, q2, q3, q4, q5
