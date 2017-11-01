# coding: utf-8
#!/usr/bin/env python3
"""
    Dynamic model of KUKA YouBot Arm.
"""
import datetime as dt
import re
import time
import sympy
from sympy import *

N = 5
G = 9.82
G0 = Matrix([[0], [0], [-G]])
O = Matrix([0, 0, 0])


class Kinematics:

    def __init__(self):
        # DH parameters 0..4
        a = symbols("a_1 a_2 a_3 a_4 a_5")
        d = symbols("d_1 d_2 d_3 d_4 d_5")
        self.ai = [a[0], a[1], a[2], 0, 0]
        self.di = [d[0], 0, 0, 0, d[4]]
        self.alphai = [pi / 2, 0, 0, pi / 2, 0]
        self.thi = [q[0], q[1], q[2], q[3], q[4]]

    def T(i, j, simp=False):
        "homogeneus matrices, i>j"
        if Ts[i][j] != -1:
            return Ts[i][j]
        else:
            H = eye(4)
            for k in range(i, j):
                Rz = Matrix(
                    [[cos(thi[i]), -sin(thi[i]), 0, 0], [sin(thi[i]), cos(thi[i]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                Rx = Matrix(
                    [[1, 0, 0, 0], [0, cos(alphai[i]), -sin(alphai[i]), 0], [0, sin(alphai[i]), cos(alphai[i]), 0],
                     [0, 0, 0, 1]])
                Tz = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, di[i]], [0, 0, 0, 1]])
                Tx = Matrix([[1, 0, 0, ai[i]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                dT = Rz * Tz * Tx * Rx
                H = H * dT
            if simp:
                H = mySimple(H)
            Ts[i][j] = H
            return H


    def test(self):
        print(t)

if __name__ == '__main__':
    lm = LinearModel()
    lm.test()