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
import numpy as np
from xi import XI
from plot_stuff import plot, plotTaus
import matplotlib.pyplot as plt
import time


class Identification:

    def __init__(self, a, d):
        self.xi = XI()

        # DH parameters
        self.a = a
        self.d = d

    def getXiExNum(self, q, dq, ddq, reshape=0, compress=1):
        """
            q, dq, ddq -- angle, velocity, acceleration;
            a, d -- DH-parameters;
            :returns computed extended overline{xi} matrix in numerical form.
        """
        a = self.a
        d = self.d

        elementsOfRegressor = [i for i in range(5)]
        # needCols = [[11, 12, 13],
        #             [0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13],
        #             [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13],
        #             [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13],
        #             [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]]
        # needCols = [[11, 12, 13],  # 0,1,2,3,4,5,6,7,8,9,10, 18, 19, 32,46
        #             [0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13],  # 4,5
        #             [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13],  # 4
        #             [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13],  # 4
        #             [0]]
        # 0,1,2,3,4,5,6,7,8,9,10, 18,19, 32, 46
        needCols = [[11, 12, 13],  # 0,1,2,3,4,5,6,7,8,9,10, 18, 19, 32,46
                    [0, 1, 2, 3,       6, 7, 8, 9, 10, 11, 12, 13],           # 4,5
                    [0, 1, 2, 3,    5, 6, 7, 8, 9, 10, 11, 12, 13],        # 4
                    [0, 1, 2, 3,    5, 6, 7, 8, 9, 10, 11, 12, 13],        # 4
                    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]]

        xi = self.xi.getXiNum_with_zeros(q, dq, ddq, a, d)
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

        for i in range(n):
            for j in range(n):
                x = list(xi[i][j])
                new_x = []
                for k in range(14):
                    if k in needCols[j]:
                        new_x.append(x[k])
                xi[i][j] = new_x

        # print('regressor was extended!')
        new_xi = []
        if reshape:
            for i in range(n):
                row = []
                for j in range(n):
                    row = row + xi[i][j]
                new_xi.append(row)
            return new_xi
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
        startTime = time.time()
        n = len(qs)
        nR = 5
        big_xi = []
        for k in range(n):
            xi = self.getXiExNum(qs[k], dqs[k], ddqs[k])
            # TODO NOT TESTED
            for i in range(nR):
                x = []
                for j in range(nR):
                    x.extend(xi[i][j])
                big_xi.append(x)
            print(str(k) + ' regressor from '+ str(n) +' was added to big_xi!')
        endTime = time.time()
        deltaTime = time.time()
        print(self.printWastedTime(startTime, endTime, deltaTime,
                             "Making big Xi:"))
        return big_xi

    def computeDDq(self, dq_prev, dq_next, t_prev, t_next):
        ddq = []
        for i in range(5):
            lim = (dq_next[i] - dq_prev[i]) / (t_next - t_prev)
            ddq.append(lim)
        return ddq

    def fltr(self, s, length=101):
        n = len(s)
        f = int(length / 2)
        for j in range(5):
            for i in range(f, n - f):
                sum = s[i][j]
                for k in range(1, f+1):
                    sum += s[i-k][j]
                    sum += s[i+k][j]
                s[i][j] = sum / length
        return s


    def readIdentData(self, fileName, filter=0):
        """
        data measures from real manipulator;
        :return: q, dq, ddq, tau_e -- matrices of vectors
        """
        file = open(fileName, 'r')
        qs, dqs, ddqs, taus, ts = [], [], [], [], []
        ddqs.append([0., 0., 0., 0., 0.])    # first ddq
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
            if i > 1:
                dq_prev = dqs[i-2]
                dq_next = dqs[i]
                t_prev = ts[i-2]
                t_next = ts[i]
                ddq = self.computeDDq(dq_prev, dq_next, t_prev, t_next)
                ddqs.append(ddq)

        print('data from ' + fileName + ' was read!')
        if filter:
            ddqs = []
            qs = self.fltr(qs)
            dqs = self.fltr(dqs)
            taus = self.fltr(taus)
            for i in range(len(qs)):
                if i > 1:
                    dq_prev = dqs[i - 2]
                    dq_next = dqs[i]
                    t_prev = ts[i - 2]
                    t_next = ts[i]
                    ddq = self.computeDDq(dq_prev, dq_next, t_prev, t_next)
                    ddqs.append(ddq)
            ddqs.append([0, 0, 0, 0, 0])
            ddqs = self.fltr(ddqs)
            print("filtering was compete!")

        file.close()
        ddqs.append([0, 0, 0, 0, 0])
        return qs, dqs, ddqs, taus, ts

    def readIdentDataFilt(self, fileName, filter=0):
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
            ddq = list(map(float, raw[10:15]))
            tau = list(map(float, raw[15:20]))
            t = float(raw[20])
            qs.append(q)
            dqs.append(dq)
            ddqs.append(ddq)
            taus.append(tau)
            ts.append(t)
        print(len(qs))
        print(len(dqs))
        print(len(ddqs))
        print(len(taus))
        print(len(ts))
        return qs, dqs, ddqs, taus, ts

    def printWastedTime(self, st, et, dt, title=''):
        prepString = title + '\nStart time: {0}\nEnd time: {1}\n Wasted time: {2}'
        start = time.strftime("%b %d %H:%M:%S", time.localtime(st))
        end = time.strftime("%b %d %H:%M:%S", time.localtime(et))
        delta = time.strftime("%b %d %H:%M:%S", time.localtime(dt - 3600 * 24)) # -3 hours for 1970 year
        wasted = prepString.format(start, end, delta)
        return wasted

    def writeBigXiFiles(self, q, dq, ddq, ident):
        for i in range(1, 2):
            bigXi = ident.makeBigXi(qs, dqs, ddqs)
            print('bit xi len: ' + str(len(bigXi)))
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

    def writeEE(self,  ident):
        for i in range(100):
            q = numpy.random.rand(5) * 4 - 2
            dq = numpy.random.rand(5) * 4 - 2
            ddq = numpy.random.rand(5) * 4 - 2
            xi = ident.getXiExNum(q, dq, ddq)
            f = open('EE/EE' + str(i) + '.txt', 'w')
            s = []
            for i in range(5):
                sr = xi[i][0]
                for j in range(1, 5):
                    sr = sr + xi[i][j]
                s.append(sr)
            s = scipy.matrix(s)
            s = numpy.transpose(s).tolist()
            for i in range(len(s)):
                l = ' '.join(str(el) for el in s[i])
                f.write(l + '\n')
            f.close()
        print('Success!')

if __name__ == '__main__':

    a = (0.033, 0.155, 0.135, 0., 0.)
    d = (0.147, 0, 0, 0, 0.218)
    ident = Identification(a, d)
    # ident.writeEE(ident)


    # q = []
    # dq = []
    # ddq = []
    #
    # x = XI()
    # q = (-0.003362, - 0.000146,0.000535, 0.001290,0.004798)
    # dq = (0.001441, -0.000631, - 0.008836,0.051949,0.001979)
    # ddq = (- 0.255993,0.248325, - 0.071348,1.710817,0.779999)
    #
    # mx_xi = ident.getXiExNum(q, dq, ddq)
    #
    # print(mx_xi[0])
    # print(mx_xi[1])
    # print(mx_xi[2])
    # print(mx_xi[3])
    # print(mx_xi[4])
    #
    # """filtering test"""
    # a = (0.033, 0.155, 0.135, 0., 0.)
    # d = (0.147, 0, 0, 0, 0.218)
    # ident = Identification(a, d)
    #
    # # first version
    i = 1
    fileName = 'data_for_identification/data_' + str(i) +'.txt'
    qs, dqs, ddqs, taus, ts = ident.readIdentDataFilt(fileName)
    print(len(qs))
    print(len(dqs))
    print(len(ddqs))
    print(len(taus))
    plot(qs, dqs, ddqs, taus, ts)
    ident.writeBigXiFiles(qs, dqs, ddqs, ident)

    # i = 1
    # fileName = 'data_for_identification/data_' + str(i) +'.txt'
    # qs, dqs, ddqs, taus, ts = ident.readIdentData(fileName)
    # qs = numpy.column_stack((qs, dqs))
    # qs = numpy.column_stack((qs, ddqs))
    # qs = numpy.column_stack((qs, taus))
    # qs = numpy.column_stack((qs, ts))
    #
    # numpy.savetxt('data_1.txt', qs)


    # """estemation vector and graphics xi"""
    # chis = []
    # for i in range(1, 2):
    #     f = open('data_for_identification/processed_data/big_tau' + str(i) + '.txt', 'r')
    #     tau = []
    #     for line in f:
    #         s = line.replace('\n', '').split(' ')
    #         l = list(map(float, s))
    #         tau.append(l)
    #     f.close()
    #     t = []
    #     for j in range(len(tau)):
    #         t = t + tau[j]
    #
    #     f = open('data_for_identification/processed_data/big_xi' + str(i) + '.txt', 'r')
    #     big = []
    #     for line in f:
    #         s = line.replace('\n', '').split(' ')
    #         l = list(map(float, s))
    #         big.append(l)
    #     f.close()
    #
    #     t = matrix(t)
    #     big = matrix(big)
    #
    #     chi = (numpy.transpose(big) * big)**(-1) * numpy.transpose(big) * numpy.transpose(t)
    #     chis.append(numpy.transpose(chi).tolist())
    # # print(chis)
    # """
    #     1) найти оценку вектора хи для первого файла
    #     2) для каждой строчки из второго файла:
    #         - найти расширенный регрессор
    #         - помножить на оценку вектора хи
    #         - получим моменты для каждого из звеньев для кучи измерений q
    #     3) построить график для моментов из первого файла
    #     4) построить график для расчитаных моментов
    # """
    # est_chi = matrix(chis[0][0])
    #
    # big = np.array(big)
    # est_chi = np.array(est_chi)
    #
    # calc_taus = np.array(big.dot(np.transpose(est_chi)))
    # calc_taus = np.array(np.hsplit(np.transpose(calc_taus)[0], len(calc_taus) / 5))
    # print(calc_taus.shape)
    #
    # i = 1
    # fileName = 'data_for_identification/data_' + str(i) + '.txt'
    # qs, dqs, ddqs, taus, ts = ident.readIdentDataFilt(fileName)
    # # plot(qs,dqs,ddqs,taus, ts)
    # # print(len(qs))
    # # print(len(dqs))
    # # print(len(ddqs))
    # # calc_taus = []
    # # for i in range(int(len(qs))):
    # #     some_xi = ident.getXiExNum(qs[i], dqs[i], ddqs[i], reshape=1)
    # #     print('progress: ' + str(i * 100.0 / len(qs)) + '%')
    # #     some_tau = numpy.transpose(matrix(some_xi) * numpy.transpose(est_chi))
    # #     calc_taus.append((some_tau.tolist())[0])
    # #
    #
    # plotTaus(taus, calc_taus, ts, ts)

    #
