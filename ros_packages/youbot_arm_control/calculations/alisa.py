#!/usr/bin/env python3
import numpy as np

from identification import Identification
from libs.plot_stuff import plotTaus2, plotTaus

A = (0.033, 0.155, 0.135, 0., 0.)
D = (0.147, 0, 0, 0, 0.218)

PATH = 'data_for_identification'

DATA_FILE_NAME = PATH + '/data/data_{:d}_filt.txt' # 'data_{:d}_filt.txt'
BIG_TAUs_FILE_NAME = PATH + '/bigs/big_tau{:d}.txt'
BIG_XIs_FILE_NAME = PATH + '/bigs/big_xi{:d}.txt'
EE_FILE_NAME = PATH + '/ee/EE{:d}.txt'

if __name__ == '__main__':
    ident = Identification(A, D)

    for i in range(1, 6):
        datafileName = DATA_FILE_NAME.format(i)
        bigXiFileName = BIG_XIs_FILE_NAME.format(i)
        bigTauFileName = BIG_TAUs_FILE_NAME.format(i)

        # # compute matrix
        Q, dQ, ddQ, Tau, T = ident.readIdentData(datafileName)

        # ident.writeEE(EE_FILE_NAME, 50)

        bigXi = ident.getBigXi(Q, dQ, ddQ)
        bigTau = ident.getBigTau(Tau)

        ident.writeBigXi(bigXiFileName, bigXi)
        ident.writeBigTau(bigTauFileName, bigTau)

    # read matrix
    # bigXi = ident.readBigXi(bigXiFileName)
    #bigTau = ident.readBigTau(bigTauFileName)

    # bigXisLite = ident.getBigXisLite(bigXi)
    # bigTausLite = ident.getBigTausLite(Tau)
    # estChiLite = ident.getEstChisLite(bigXisLite,bigTausLite)
    # caltTaus = ident.getCalcTausLite(bigXisLite, estChiLite)
    #
    # print(caltTaus.shape)
    #
    # plotTaus2(Tau, caltTaus, T, T)

    # print(bigXi.shape)
    # bigTau = np.hstack(bigTau)
    # print(bigTau.shape)
    #
    # bigXiT = np.transpose(bigXi)
    #
    # # prepareBigXi = np.linalg.inv(bigXiT.dot(bigXi)).dot(bigXiT)
    # # estChi = prepareBigXi *(bigTau)
    #
    # prepareBigXi = np.matmul(np.linalg.inv(np.matmul(bigXiT, bigXi)), bigXiT)
    # estChi = np.matmul(prepareBigXi, bigTau)
    #
    # print(len(estChi))
    #
    # estTau = bigXi.dot(estChi)
    #
    # estTau = np.array(np.hsplit(estTau, len(estTau) / 5))
    #
    # print(estTau)
    #
    # print(estTau.shape)
    # print(np.array(Tau).shape)
    # Tau = np.array(Tau)
    #
    # plotTaus2(Tau, estTau, T, T)

