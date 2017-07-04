#!/usr/bin/env python3
"""
    Collect xi matrix (regressor).
"""
import numpy as np
# from alisa import A, D
from libs.utils import getZeros

from compute_xi_v3_simp.xi.xi_00 import XI as XI00
from compute_xi_v3_simp.xi.xi_01 import XI as XI01
from compute_xi_v3_simp.xi.xi_02 import XI as XI02
from compute_xi_v3_simp.xi.xi_03 import XI as XI03
from compute_xi_v3_simp.xi.xi_04 import XI as XI04

from compute_xi_v3_simp.xi.xi_11 import XI as XI11
from compute_xi_v3_simp.xi.xi_12 import XI as XI12
from compute_xi_v3_simp.xi.xi_13 import XI as XI13
from compute_xi_v3_simp.xi.xi_14 import XI as XI14

from compute_xi_v3_simp.xi.xi_22 import XI as XI22
from compute_xi_v3_simp.xi.xi_23 import XI as XI23
from compute_xi_v3_simp.xi.xi_24 import XI as XI24

from compute_xi_v3_simp.xi.xi_33 import XI as XI33
from compute_xi_v3_simp.xi.xi_34 import XI as XI34

from compute_xi_v3_simp.xi.xi_44 import XI as XI44

N_L = 10


class XIFull:
    """ regressor """

    def __init__(self, a, d):
        xi0 = [XI00(a=a, d=d),  XI01(a=a, d=d), XI02(a=a, d=d), XI03(a=a, d=d), XI04(a=a, d=d)]
        xi1 = [             0,  XI11(a=a, d=d), XI12(a=a, d=d), XI13(a=a, d=d), XI14(a=a, d=d)]
        xi2 = [             0,               0, XI22(a=a, d=d), XI23(a=a, d=d), XI24(a=a, d=d)]
        xi3 = [             0,               0,              0, XI33(a=a, d=d), XI34(a=a, d=d)]
        xi4 = [             0,               0,              0,              0, XI44(a=a, d=d)]
        self.xi = [xi0, xi1, xi2, xi3, xi4]
        self.linerCols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 17, 18, 20, 28, 31, 32, 42, 44, 45, 46,
                          15, 14]
        # zeros [2, 3, 4, 6, 7, 8, 9]

    def getWellColNums(self):
        all = set(range(70))
        well = all.difference(self.linerCols)
        return list(well)

    def getXiNum(self, q, dq, ddq):
        """ :returns Standart not extended regressor 5x50 in numerical form """
        n = len(q)
        xi = np.empty(0)
        for i in range(n):
            for j in range(n):
                if self.xi[i][j] != 0:
                    xi_ij = self.xi[i][j].getXI(q, dq, ddq)
                    xi = np.concatenate((xi, xi_ij))
                else:
                    xi = np.concatenate((xi, np.zeros(N_L)))
        return xi.reshape(n, n * N_L)

    def getXiNumEx(self, q, dq, ddq):
        """ :returns Extended regressor 5x5*14 in numerical form """
        n = len(q)
        xi = self.getXiNum(q, dq, ddq)
        basePart = xi.reshape(n * n, N_L)
        extendPart = np.empty(0)    # 1 x 100
        for i in range(n):
            for j in range(n):
                if i == j:
                    extendPart = np.concatenate((extendPart, [ddq[j], dq[j], np.sign(dq[j]), 1]))
                else:
                    extendPart = np.concatenate((extendPart, [0, 0, 0, 0]))
        extendPart = extendPart.reshape(n * n, 4)
        extendXi = np.concatenate((basePart, extendPart), axis=1)
        return extendXi.reshape(n, n * (N_L + 4))

    def getXiNumExCompressed(self, q, dq, ddq):
        """ :returns Compressed extended regressor 5x5*~? in numerical form """
        n = len(q)
        xi = self.getXiNumEx(q, dq, ddq)
        cXi = np.delete(xi, self.linerCols, 1)
        return cXi


if __name__ == '__main__':
    A = (0.033, 0.155, 0.135, 0., 0.)
    D = (0.147, 0, 0, 0, 0.218)
    xiFull = XIFull(A, D)

    q = (0,0,0,0,0)
    dq = (0, 0, 0, 0, 0)
    ddq = (0, 0, 0, 0, 0)

    xi = xiFull.getXiNumExCompressed(q, dq, ddq)
    print(xi.shape)
    for i in range(5):
        print(xi[i].tolist())
