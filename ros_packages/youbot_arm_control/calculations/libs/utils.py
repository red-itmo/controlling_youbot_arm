#!/usr/bin/env python3
import numpy as np
import time

EPS = 1.e-15

def printWastedTime(st, et, dt, title=''):
    prepString = title + '\nStart time: {0}\nEnd time: {1}\nWasted time: {2}'
    start = time.strftime("%b %d %H:%M:%S", time.localtime(st))
    end = time.strftime("%b %d %H:%M:%S", time.localtime(et))
    delta = time.strftime("%b %d %H:%M:%S", time.localtime(dt - 3600 * 24))  # -3 hours for 1970 year
    wasted = prepString.format(start, end, delta)
    print(wasted)


# def rmZeros(xi_l):
#     n = len(xi_l)
#     zeros = np.empty(0)
#     for i in range(n):
#         if xi_l[i] == 0:
#             zeros = np.append(zeros, i)
#     xi_l = np.delete(xi_l, zeros.tolist(), 1)
#     return xi_l


def getZeros(mx):
    """
        axis in [1, 2]
            1 -- rows
            2 -- cols
    :param mx:
    :param axis:
    :return:
    """
    m = len(mx[0])
    zeros = []
    for j in range(m):
        if np.all(mx[:, j] < EPS):
            zeros.append(j)
    return zeros


def rmZeros(mx):
    zeroCols = getZeros(mx)
    mx = np.delete(mx, zeroCols, 1)
    return mx

if __name__ == '__main__':
    a = np.array([[1,2],[3,4],[5,6]])
    b = np.zeros((3, 1))
    w = np.concatenate((a,b,b,a,b), 1)
    print(w)
    print(getZeros(w))
    print(rmZeros(w))