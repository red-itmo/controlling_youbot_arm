# coding: utf-8
#!/usr/bin/env python3

import datetime as dt

import time

import sympy
from sympy import *
import re
# from symengine import cos, sin
# from sympy import simplify, transpose, Matrix, symbols, pi, eye

# from sympy.interactive import printing
# printing.init_printing(use_latex='mathjax')


n = 5 # i = 0..4
g = 9.82
g0 = Matrix([[0], [0], [-g]])
"initialization of arrays for frequently use matrices"
TENSORs = [-1 for j in range(n + 1)]
Ts = [[-1 for j in range(n + 1)] for i in range(n + 1)]
Rs = [[-1 for j in range(n + 1)] for i in range(n + 1)]
Zs = [-1 for j in range(n + 1)]

# jacobians
O = Matrix([0, 0, 0])
Jv = [[O for j in range(n)] for i in range(n)]
Jomega = [[O for j in range(n)] for i in range(n)]

# START Q BLOCK
# FIRST alternative way for "q" as just symbols
# q = symbols("q_1 q_2 q_3 q_4 q_5")
# dq = symbols('\dot{q_1} \dot{q_2} \dot{q_3} \dot{q_4} \dot{q_5}')
# ddq = symbols('\ddot{q_1} \ddot{q_2} \ddot{q_3} \ddot{q_4} \ddot{q_5}')

# SECOND alternative way for "q" as functions q_i(t)
t = Symbol('t')
q = list()
dq = list()
ddq = list()
for i in range(0, n):
    q.append(Function('q_' + str(i + 1))(t))
    dq.append(diff(q[i], t))
    ddq.append(diff(q[i], t, 2))
# END Q BLOCK

# DH parameters 0..4
a = symbols("a_1 a_2 a_3 a_4 a_5")
d = symbols("d_1 d_2 d_3 d_4 d_5")
ai = [a[0], a[1], a[2], 0, 0]
di = [d[0], 0, 0, 0, d[4]]
alphai = [pi / 2, 0, 0, pi / 2, 0]
thi = [q[0], q[1]+pi/2, q[2], q[3], q[4]]

# init mass 0..4
m = symbols('m_1 m_2 m_3 m_4 m_5')
"VARIABLE BELOW USING FOR LATEX GENERATION"
# vars for radius vectors 0..5
xc = symbols('x_{c0} x_{c1} x_{c2} x_{c3} x_{c4} x_{c5}')
yc = symbols('y_{c0} y_{c1} y_{c2} y_{c3} y_{c4} y_{c5}')
zc = symbols('z_{c0} z_{c1} z_{c2} z_{c3} z_{c4} z_{c5}')
x = symbols('x_{0} x_{1} x_{2} x_{3} x_{4} x_{5}')
y = symbols('y_{0} y_{1} y_{2} y_{3} y_{4} y_{5}')
z = symbols('z_{0} z_{1} z_{2} z_{3} z_{4} z_{5}')
# init moments 0..4
Ixx = symbols('I_{1_{xx}} I_{2_{xx}} I_{3_{xx}} I_{4_{xx}} I_{5_{xx}}')
Iyy = symbols('I_{1_{yy}} I_{2_{yy}} I_{3_{yy}} I_{4_{yy}} I_{5_{yy}}')
Izz = symbols('I_{1_{zz}} I_{2_{zz}} I_{3_{zz}} I_{4_{zz}} I_{5_{zz}}')
Ixy = symbols('I_{1_{xy}} I_{2_{xy}} I_{3_{xy}} I_{4_{xy}} I_{5_{xy}}')
Ixz = symbols('I_{1_{xz}} I_{2_{xz}} I_{3_{xz}} I_{4_{xz}} I_{5_{xz}}')
Iyz = symbols('I_{1_{yz}} I_{2_{yz}} I_{3_{yz}} I_{4_{yz}} I_{5_{yz}}')

"ALTERNATIVE SYMBOLIC VARIABLES FOR EASIER CODE GENERATION. USING INSTEAD WHICH ARE ABOVE"
xc = symbols('xc0 xc1 xc2 xc3 xc4 xc5')
yc = symbols('yc0 yc1 yc2 yc3 yc4 yc5')
zc = symbols('zc0 zc1 zc2 zc3 zc4 zc5')
x = symbols('x0 x1 x2 x3 x4 x5')
y = symbols('y0 y1 y2 y3 y4 y5')
z = symbols('z0 z1 z2 z3 z4 z6')
Ixx = symbols('I1xx I2xx I3xx I4xx I5xx')
Iyy = symbols('I1yy I2yy I3yy I4yy I5yy')
Izz = symbols('I1zz I2zz I3zz I4zz I5zz')
Ixy = symbols('I1xy I2xy I3xy I4xy I5xy')
Ixz = symbols('I1xz I2xz I3xz I4xz I5xz')
Iyz = symbols('I1yz I2yz I3yz I4yz I5yz')


# MATRICES and additional stuff-
def mySimple(expr):
    "how simpify"
    expr = simplify(expand(expr))
    return expr


def T(i, j, simp=False):
    """
        T(0,1) is equal to {}^0 A_1
    """
    if Ts[i][j] != -1:
        return Ts[i][j]
    else:
        H = eye(4)
        for k in range(i, j):
            Rz = Matrix(
                [[cos(thi[k]), -sin(thi[k]), 0, 0],
                 [sin(thi[k]), cos(thi[k]), 0, 0],
                 [0, 0, 1, 0], [0, 0, 0, 1]])
            Rx = Matrix([[1, 0, 0, 0],
                         [0, cos(alphai[k]), -sin(alphai[k]), 0],
                         [0, sin(alphai[k]), cos(alphai[k]), 0],
                         [0, 0, 0, 1]])
            Tz = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, di[k]], [0, 0, 0, 1]])
            Tx = Matrix([[1, 0, 0, ai[k]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            dT = Rz * Tz * Tx * Rx
            H = H * dT
        if simp:
            H = mySimple(H)
        Ts[i][j] = H
        return H


def R(i, j, simp=False):
    "rotation matrices, i>j"
    if Rs[i][j] != -1:
        return Rs[i][j]
    else:
        r = T(i, j)[:3, :3]
        if simp:
            r = mySimple(r)
        Rs[i][j] = r
        return r


def get_z(i, simp=True):
    """
        get_z(1) = z^0_1
    """
    if Zs[i] != -1:
        return Zs[i]
    else:
        z = R(0, i) * Matrix([[0], [0], [1]])
        Zs[i] = z
        return z


def get_r0_0To(i):
    """
        get_r0_0To(1) = r^0_{0,1}
    """
    r = T(0, i)[:3, 3]
    return r  # 3x1


def get_ri_0To(i, simp=False):
    """
        get_ri_0To(1) = r^1_{0, 1}
    """
    r = transpose(R(0, i)) * get_r0_0To(i)
    if simp:
        r = mySimple(r)
    return r  # 3x1


def get_g(i, simp=False):
    """
        get_g(1) = g_1
    """
    g = transpose(R(0, i)) * g0
    if simp:
        g = mySimple(g)
    return g  # 3x1


# INIT JACOBIANS
# for v
for i in range(0, n):
    for j in range(0, i + 1):
        el = get_z(j).cross(get_r0_0To(i+1) - get_r0_0To(j))
        Jv[i][j] = el


def getJv(i):
    """
        getJv(0) = J_{v1}
    """
    j = Matrix(Jv[i])
    j = j.reshape(5, 3)
    return transpose(j)

# for omega
for i in range(0, n):
    for j in range(0, i + 1):
        el = get_z(j)
        Jomega[i][j] = el


def getJomega(i):
    """
        getJomega(0) = J_{omega1}
    """
    j = Matrix(Jomega[i])
    j = j.reshape(5, 3)
    return transpose(j)


def get_v0(i, simp=False):
    """
        get_v0(1) = v^0_1
    """
    v = getJv(i-1) * Matrix(dq)
    return v


def get_vi(i, simp=False):
    """
        get_vi(1) = v^1_1
    """
    v = transpose(R(0, i)) * get_v0(i)
    if simp:
        v = mySimple(v)
    return v  # 3x1


def get_omega0(i, simp=False):
    """
        get_omega0(1) = omega^0_1
    """
    omega = getJomega(i-1) * Matrix(dq)
    return omega


def get_omegai(i, simp=False):
    """
        get_omegai(1) = omega^1_1
    """
    omega = transpose(R(0, i)) * get_omega0(i)
    if simp:
        omega = mySimple(omega)
    return omega  # 3x1


# COMPUTE LAGRANGIANINKI
nL = 10
L = [[-1 for i in range(0, nL)] for j in range(0, n)]
tm = time.time
totalT = tm()
print('start computig of L')
for i in range(0, n):
    """cols(L) = 0..9, rows(L) = 0..4"""
    startTrow = tm()
    l24 = get_vi(i+1).cross(get_omegai(i+1)) + get_g(i+1)
    omegai = get_omegai(i+1)
    L[i][0] = Rational(1, 2) * transpose(get_vi(i+1)) * get_vi(i+1) + transpose(get_g(i+1)) * get_ri_0To(i+1)
    # L[i][0] = Rational(1,2) * transpose(get_v0(i)) * get_v0(i) + transpose(g0) * get_r0_0To(i)
    L[i][1] = l24[0]
    L[i][2] = l24[1]
    L[i][3] = l24[2]
    L[i][4] = Rational(1, 2) * omegai[0] ** 2
    L[i][5] = Rational(1, 2) * omegai[1] ** 2
    L[i][6] = Rational(1, 2) * omegai[2] ** 2
    L[i][7] = omegai[0] * omegai[1]
    L[i][8] = omegai[0] * omegai[2]
    L[i][9] = omegai[1] * omegai[2]
    for j in range(0, nL):
        startTel = tm()
        # L[i][j] = mySimple(L[i][j])
        print(str(tm() - startTel) + ': j= ' + str(j) + 'is OK!')
    print(str(tm() - startTrow) + ': i= ' + str(i) + ' row of L was computed!')
totalT = tm() - totalT
print('!total time: ' + str(totalT))


# differentiation
def operatorL(L, j, simp=False):
    """
        operatorL(L, 0) = ~~~~q_1
    """
    dL_Ddq = diff(L, dq[j])
    dLdq_Dt = diff(dL_Ddq, t)
    dL_Dq = diff(L, q[j])
    opL = dLdq_Dt - dL_Dq
    if simp:
        opL = mySimple(opL)
    return opL

# clear code for scilab
def octaveButyfiler(expr):
    ptq = r"[d]*q_[1-5]"
    ptex = r"Derivative\([d]*q_[1-5]+\(t\),[\w ,]+\)"
    ptt = r"\(t\)"
    pt = r"[t]+"

    def repldq(m):
        lala = re.search(ptq, m.group(0))
        t = re.findall(pt, m.group(0))
        if len(t) - 2 == 1:
            return "d" + lala.group(0)
        elif len(t) - 2 == 2:
            return "dd" + lala.group(0)

    def rmt(m):
        return ""

    rerdq = re.sub(ptex, repldq, expr)
    remt = re.sub(ptt, rmt, rerdq)
    return remt


def python_gencode(expr):
    """ python code cleaner"""

    def replaceDotsQ(m):
        "replace Derivativ and replace q_i(t) on dq_i and next dq[i]"
        lala = re.search(r"[d]*q_[1-5]", m.group(0))
        t = re.findall(r"[t]+", m.group(0))
        num = re.search(r"[1-5]", lala.group(0))
        n = int(num.group(0))-1
        if len(t) - 2 == 1:
            return "dq[" + str(n) + "]"
        elif len(t) - 2 == 2:
            return "ddq[" + str(n) + "]"

    def replaceADi(m):
        paramName = re.search(r"[ad]", m.group(0))
        num = re.search(r"[1-5]", m.group(0))
        n = int(num.group(0))-1
        return str(paramName.group(0)) + "[" + str(n) + "]"

    def deleteT(m):
        return ""

    def replaceQi(m):
        q_i = re.search(r"q_[1-5]", m.group(0))
        num = re.search(r"[1-5]", m.group(0))
        n = int(num.group(0))-1
        return "q[" + str(n) + "]"

    # replace all Derivative(.*)
    replace_dotsq = re.sub(r"Derivative\([d]*q_[1-5]+\(t\),[\w ,]+\)", replaceDotsQ, expr)
    # delete all "(t)"
    replace_t = re.sub(r"\(t\)", deleteT, replace_dotsq)
    # replace all q_i to q[i]
    replace_qi = re.sub(r"q_[1-5]", replaceQi, replace_t)
    # replace DH parameters
    replace_adi = re.sub(r"[ad]_[1-5]", replaceADi, replace_qi)
    return replace_adi


def makeXImodules():
    """
        1) Compute 150 components of regressor;
        2) Set it to modules.
        An example of the generated module see in example_module.py

        Function create two directories that contain files with expressions for
        Lagrange-equations without parameters (mass, inertia, friction).
        - <<xi>> -- expressions in sympy form;
        - <<xi_tex>> -- expressions in latex form.
    """
    regressor_zeros = [[-1 for j in range(50)] for i in range(5)]
    tm = time.time
    startTmatrix = tm()
    print('computing start! ' + str(dt.datetime.fromtimestamp(tm())))
    for j in range(0, n):  # 0..n
        startTrow = tm()
        print('\trow_' + str(j) + ' start!')
        for i in range(j, n):  # j..n
            print('\t\telement_' + str(i) + ' start!')
            fileName = 'xi/xi_' + str(j) + str(i) + '.py'
            fileName_tex = 'xi_tex/xi_' + str(j) + str(i)

            file = open(fileName, 'w')
            # first piece of template of modul
            module = '#!/usr/bin/env python3\n'
            module += 'from numpy import cos, sin\n'
            module += '\n\n'
            module += 'class XI:\n'
            module += '\t"""XI_' + str(i) + str(j) + '"""\n\n'
            module += '\tdef __init__(self, q=(0,0,0,0,0), dq=(0,0,0,0,0), ' \
                      'ddq=(0,0,0,0,0), a=(0,0,0,0,0), d=(0,0,0,0,0)):\n' \
                      '\t\tself.q, self.dq, self.ddq = q, dq, ddq\n' \
                      '\t\tself.a, self.d = a, d\n\n'
            module += '\tdef setData(self, q, dq, ddq, a, d):\n' \
                      '\t\tself.q, self.dq, self.ddq = q, dq, ddq\n' \
                      '\t\tself.a, self.d = a, d\n\n'
            file.write(module)
            file.close()

            for k in range(0, nL):
                startTelement = tm()
                print('\t\t\tsubElement_' + str(k) + ' start at ' + str(dt.datetime.fromtimestamp(tm())))

                file = open(fileName, 'a')
                file_tex = open(fileName_tex, 'a')

                # compute expression
                opL_sym_raw = operatorL(L[i][k], j)
                print(type(opL_sym_raw))
                if type(opL_sym_raw) in [ImmutableMatrix]:
                    opL_sym_raw = opL_sym_raw[0]
                len_raw = len(str(opL_sym_raw))
                print(len_raw)

                # simplify expression
                opL_sym = combsimp(powsimp(trigsimp(expand(opL_sym_raw))))
                # opL_sym = simplify(opL_sym)   # alternative method

                if opL_sym == sympy.numbers.Zero():
                    regressor_zeros[j][(i*10) + k] = 0

                len_ok = len(str(opL_sym))
                print(len_ok)
                time.sleep(1)

                # if simplify was shit ;)
                if len_raw < len_ok:
                    opL_sym = opL_sym_raw

                " START CREATING method in MODULE "

                # generate python code
                opL_py_raw = sympy.printing.lambdarepr.lambdarepr(opL_sym)

                # some replaces, e.g. a_1 to a[0], Derivative(q_1(t), t) to dq[0]
                opL_py_well = python_gencode(opL_py_raw)

                opL_sym = opL_py_well
                opNum = str(k)
                # aaaaand template ooof module

                # second piece of template of modul
                # methods contains computing operator L
                module = '\tdef opL' + opNum + '(self):\n' \
                            '\t\t"""' + str(i) + str(j) + str(k) + '"""\n' \
                            '\t\tq, dq, ddq = self.q, self.dq, self.ddq\n' \
                            '\t\ta, d = self.a, self.d\n' \
                            '\t\topL_' + opNum + ' = ' + opL_sym + '\n' \
                            '\t\treturn opL_' + opNum + '\n\n'

                " END CREATING method in MODULE "

                # writing data to files
                file_tex.write(str(opL_sym))
                file_tex.close()

                file.write(module)
                file.close()

                print('\t\t\tsubElement_' + str(k) + ': ' + str(tm() - startTelement) + ' end at ' + str(
                    dt.datetime.fromtimestamp(tm())))
            file = open(fileName, 'a')
            # third piece of template of modul
            module = '\tdef getXI(self, q, dq, ddq, a, d):\n' \
                      '\t\tself.setData(q, dq, ddq, a, d)\n' \
                      '\t\tXI = [0 for i in range(10)]\n' \
                      '\t\tXI[0] = self.opL0()\n' \
                      '\t\tXI[1] = self.opL1()\n' \
                      '\t\tXI[2] = self.opL2()\n' \
                      '\t\tXI[3] = self.opL3()\n' \
                      '\t\tXI[4] = self.opL4()\n' \
                      '\t\tXI[5] = self.opL5()\n' \
                      '\t\tXI[6] = self.opL6()\n' \
                      '\t\tXI[7] = self.opL7()\n' \
                      '\t\tXI[8] = self.opL8()\n' \
                      '\t\tXI[9] = self.opL9()\n' \
                      '\t\treturn XI\n\n'
            file.write(module)
            file.close()
            print('\t\telement_' + str(i) + ': ' + str(tm() - startTrow))

        print('\trow_' + str(j) + ': ' + str(tm() - startTrow))
    print('total time on compute matrix \XI: ' + str(tm() - startTmatrix))
    print(str(dt.datetime.fromtimestamp(tm())))

    f = open('regressor_zeros.txt', 'w')
    for i in range(5):
        f.write(str(regressor_zeros[i])+'\n')
    f.close()

if __name__ == '__main__':
    # makeXImodules()
    print(T(0,3))


    "OLD VERSION OF MAKE FILE-EXPRESSIONS"
    # tm = time.time
    # startTmatrix = tm()
    # print('computing start! ' + str(dt.datetime.fromtimestamp(tm())))
    # for j in range(row, row+1): # 0..n
    #     startTrow = tm()
    #     print('\trow_' + str(j) + ' start!')
    #     # opL = ''
    #     for i in range(j, col+1): # j..n
    #         print('\t\telement_' + str(i) + ' start!')
    #         fileName = 'xi/xi_' + str(j) + str(i)
    #         fileName_tex = 'xi_tex/xi_' + str(j) + str(i)
    #         for k in range(0, nL):
    #             startTelement = tm()
    #             print('\t\t\tsubElement_' + str(k) + ' start at ' + str(dt.datetime.fromtimestamp(tm())))
    #             file = open(fileName, 'a')
    #             file_tex = open(fileName_tex, 'a')
    #
    #             opL_sym_raw = operatorL(L[i][k], j)
    #             len_raw = len(str(opL_sym_raw))
    #             print(len_raw)
    #             # opL_sym = simplify(opL_sym)
    #             opL_sym = combsimp(powsimp(trigsimp(expand(opL_sym_raw))))
    #             # opL_sym = combsimp((opL_sym_raw))
    #
    #             len_ok = len(str(opL_sym))
    #             print(len_ok)
    #             time.sleep(1)
    #             if len_raw < len_ok:
    #                 opL_sym = opL_sym_raw
    #             file_tex.write(str(opL_sym))
    #             file_tex.close()
    #
    #             varName = 'opL_' + str(i) + str(k)
    #             opL_raw = o.octave_code(opL_sym, assign_to=varName, human=True, inline=True)
    #             opL_oct = octaveButyfiler(opL_raw)
    #             opL = str(opL_oct) + '\n\n'
    #             file.write(opL)
    #             file.close()
    #             print('\t\t\tsubElement_' + str(k) + ': ' + str(tm() - startTelement) + ' end at ' + str(dt.datetime.fromtimestamp(tm())))
    #         print('\t\telement_' + str(i) + ': ' + str(tm() - startTrow))
    #     print('\trow_' + str(j) + ': ' + str(tm() - startTrow))
    # print('total time on compute matrix \XI: ' + str(tm() - startTmatrix))
    # print(str(dt.datetime.fromtimestamp(tm())))
