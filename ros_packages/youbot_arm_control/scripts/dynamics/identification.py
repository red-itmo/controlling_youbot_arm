#!/usr/bin/env python3
"""
    Collect xi matrix (regressor).
    Method of XI class getXiNum(q,dq,ddq,a,d) returns computed regressor.

    Example of using in the footer.

    \chi = [m_i, m_i x_{ci}, m_i y_{ci}, m_i z_{ci},
        I_{i, xx}, I_{i, yy}, I_{i, zz}, I_{i, xy}, I_{i, xz}, I_{i, yz},
            I_{a, i}, f_{v, i}, f_{c, i}, f_{off, i}]^T
"""
from numpy import sign
from scipy import matrix
import scipy
import numpy
from xi import XI
from plot_stuff import plot


class Identification:

    def __init__(self, a, d):
        self.xi = XI()

        # DH parameters
        self.a = a
        self.d = d

    def getXiExNum(self, q, dq, ddq):
        """
            q, dq, ddq -- angle, velocity, acceleration;
            a, d -- DH-parameters;
            :returns computed extended overline{xi} matrix in numerical form.
        """
        a = self.a
        d = self.d
        # print(q,dq,ddq,a,d)
        xi = self.xi.getXiNum(q, dq, ddq, a, d)
        n = 5
        for i in range(n):
            for j in range(n):
                if i == j:
                    x = list(xi[i][j])
                    x.extend([ddq[j], dq[j], sign(dq[j]), 1])
                    xi[i][j] = x
                else:
                    x = list(xi[i][j])
                    x.extend([0, 0, 0, 0])
                    xi[i][j] = x
        print('regressor was extended!')
        return xi

    def getTauE(self):
        """
            the torque on a shaft
        """
        pass

    def makeBigXi(self, qs, dqs, ddqs):
        """
            My name is big, my Xi is very big.
            The matrix Xi -- ((n*5), 5) of regressors for each t[i].

            Xi = [[xi_0],[xi_1],[xi_2],...,[xi_k],...,[xi_n-1]]
            when xi -- regressor for q,dq,ddq in time t.

            Xi[k][i][j]
                k -- regressor in time ts[k];
                i -- row for this regressor;
                j -- col for this regressor.

        """
        n = len(qs)
        nR = 5
        big_xi = []
        for k in range(n):
            xi = self.getXiExNum(qs[k], dqs[k], ddqs[k])
            for i in range(nR):
                x = []
                for j in range(nR):
                    x.extend(xi[i][j])
                big_xi.append(x)
            print(str(k) + ' regressor from '+ str(n) +' was added to big_xi!')
        return big_xi

    def computeDDq(self, dq, next_dq, t, next_t):
        ddq = []
        for i in range(len(dq)):
            lim = (next_dq[i] - dq[i]) / (next_t - t)
            ddq.append(lim)
        return ddq

    def readIdentData(self, fileName):
        """
        data measures from real manipulator;
        :return: q, dq, ddq, tau_e -- matrices of vectors
        """
        file = open(fileName, 'r')
        qs, dqs, ddqs, taus, ts = [], [], [], [], []
        for i, line in enumerate(file):
            raw = line.split(' ')
            q = list(map(float, raw[0:5]))
            dq = list(map(float, raw[5:10]))
            tau = list(map(float, raw[10:15]))
            t = float(raw[15])
            qs.append(q)
            dqs.append(dq)
            taus.append(tau)
            ts.append(t)
            if i > 0:
                dq0 = dqs[i - 1]
                t0 = ts[i - 1]
                ddq = self.computeDDq(dq0, dq, t0, t)
                ddqs.append(ddq)
        file.close()
        ddq_last = self.computeDDq(dqs[len(dqs) - 1], (0, 0, 0, 0, 0),
                                   ts[len(dqs) - 1], 0)
        ddqs.append(ddq_last)
        print('data from ' + fileName + ' was read!')
        return qs, dqs, ddqs, taus, ts

    def writeBigXiFiles(self):
        a = (0.133, 0.155, 0.135, 0., 0.)
        d = (0.147, 0, 0, 0, 0.218)
        ident = Identification(a, d)

        # make files for each data_file contains big_xi
        for i in range(1, 6):
            fileName = 'data_for_identification/data_' + str(i) + '.txt'
            qs, dqs, ddqs, taus, ts = ident.readIdentData(fileName)
            # plot(qs, dqs, ddqs, taus, ts)
            bigXi = ident.makeBigXi(qs, dqs, ddqs)
            f = open('data_for_identification/processed_data/big_xi' + str(i) + '.txt', 'w')
            n = len(bigXi)
            for j in range(n):
                f.write((str(bigXi[j])[1:-1]).replace(',', '') + '\n')
            f.close()

            f = open('data_for_identification/processed_data/big_tau' + str(i) + '.txt', 'w')
            n = len(taus)
            for j in range(n):
                f.write((str(taus[j])[1:-1]).replace(',', '') + '\n')
            f.close()

if __name__ == '__main__':


    chis = []
    for i in range(1,6):
        f = open('data_for_identification/processed_data/big_tau' + str(i) + '.txt', 'r')
        tau = []
        for line in f:
            s = line.replace('\n', '').split(' ')
            l = list(map(float, s))
            tau.append(l)
        f.close()
        t = []
        for j in range(len(tau)):
            t = t + tau[j]

        f = open('data_for_identification/processed_data/big_xi' + str(i) + '.txt', 'r')
        big = []
        for line in f:
            s = line.replace('\n', '').split(' ')
            l = list(map(float, s))
            big.append(l)
        f.close()

        t = matrix(t)
        big = matrix(big)

        chi = (numpy.transpose(big) * big)**(-1) * numpy.transpose(big) * numpy.transpose(t)
        chis.append(numpy.transpose(chi).tolist())
    print(chis)
