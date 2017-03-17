import math

PI = math.pi


class Config:

    DH_A = (0, 0.033, 0.155, 0.135, 0, 0)
    DH_ALPHA = (0, PI / 2, 0, 0, PI / 2, 0)
    DH_D = (0.128, 0.019, 0, 0, 0, 0.218)   # a0 = 0.096 in standart urdf
    DH_THETA = (0, 0, PI / 2, 0, 0, 0)

    def __init__(self):
        # TODO init from params?
        print('im do nothing')
