#!/usr/bin/env python3
"""
    Collect xi matrix (regressor).
    Method of XI class getXiNum(q,dq,ddq,a,d) returns computed regressor.
    Example of using in the footer.

    \chi = [m_i, m_i x_{ci}, m_i y_{ci}, m_i z_{ci},
        I_{i, xx}, I_{i, yy}, I_{i, zz}, I_{i, xy}, I_{i, xz}, I_{i, yz}]^T
"""
from from_server.compute_xi.xi.xi_00 import XI as XI00
from from_server.compute_xi.xi.xi_01 import XI as XI01
from from_server.compute_xi.xi.xi_02 import XI as XI02
from from_server.compute_xi.xi.xi_03 import XI as XI03
from from_server.compute_xi.xi.xi_04 import XI as XI04

from from_server.compute_xi.xi.xi_11 import XI as XI11
from from_server.compute_xi.xi.xi_12 import XI as XI12
from from_server.compute_xi.xi.xi_13 import XI as XI13
from from_server.compute_xi.xi.xi_14 import XI as XI14

from from_server.compute_xi.xi.xi_22 import XI as XI22
from from_server.compute_xi.xi.xi_23 import XI as XI23
from from_server.compute_xi.xi.xi_24 import XI as XI24

from from_server.compute_xi.xi.xi_33 import XI as XI33
from from_server.compute_xi.xi.xi_34 import XI as XI34

from from_server.compute_xi.xi.xi_44 import XI as XI44


class XI:
    """ regressor """

    def __init__(self):
        xi0 = [XI00(), XI01(), XI02(), XI03(), XI04()]
        xi1 = [0,      XI11(), XI12(), XI13(), XI14()]
        xi2 = [0,      0,      XI22(), XI23(), XI24()]
        xi3 = [0,      0,      0,      XI33(), XI34()]
        xi4 = [0,      0,      0,      0,      XI44()]
        self.xi_obj = [xi0, xi1, xi2, xi3, xi4]
        self.xi_zero = [0. for j in range(10)]

    def getXiNum(self, q, dq, ddq, a, d):
        """
            Compute xi matrix and remove zeros columns.

            q, dq, ddq -- angle, velocity, acceleration;
            a, d -- DH-parameters;
            :returns computed xi matrix in numerical form.
        """
        notZerosCols = [0, 1, 5]    # only for 1 operatorL
        n = 5
        xi_num = [[0 for j in range(n)] for i in range(n)]
        for i in range(n):
            for j in range(n):
                if self.xi_obj[i][j] != 0:
                    got_xi = self.xi_obj[i][j].getXI(q, dq, ddq, a, d)
                    # compress matrix
                    xi = []
                    if j == 1:
                        for k in range(10):
                            if k in notZerosCols:
                                xi.append(got_xi[k])
                    else:
                        xi = got_xi
                    xi_num[i][j] = xi
                else:
                    xi = []
                    if j == 1:
                        for k in range(10):
                            if k in notZerosCols:
                                xi.append(0)
                    else:
                        xi = self.xi_zero
                    xi_num[i][j] = xi
        return xi_num


if __name__ == '__main__':
    x = XI()
    mx_xi = x.getXiNum((0,0,2.9,-1.1,10.221),(10.2,10.9,1.6,-10.6,10.8),
                       (0,0,0,0,0),(10.1,10,10,0,0),(10,0,0,0,20))
    # print(mx_xi[0])
    # print(mx_xi[1])
    # print(mx_xi[2])
    # print(mx_xi[3])
    # print(mx_xi[4])
