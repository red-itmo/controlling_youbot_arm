def getTensor(i):
    """ getTensor(0) = I_1"""
    return Matrix([[Ixx[i], Ixy[i], Ixz[i]],
                    [Ixy[i], Iyy[i], Iyz[i]],
                    [Ixz[i], Iyz[i], Izz[i]]])

def get_riici(i):
    """ get_riici(0) = rc_1 """
    return R(0, i) * Matrix([xc[i], yc[i], zc[i]])

def getU():
    """ potential energy """
    U = 0
    for i in range(n):
        U += (m[i] * get_g(i).T * get_ri_0To(i) + get_g(i).T * (m[i] * get_riici(i)))[0]
    return U

Jx = - (getJv(i)[2,:]).T * getJomega(i)[1,:] + (getJv(i)[1,:]).T * getJomega(i)[2,:]
Jy =   (getJv(i)[2,:]).T * getJomega(i)[0,:] - (getJv(i)[0,:]).T * getJomega(i)[2,:]
Jz = - (getJv(i)[1,:]).T * getJomega(i)[0,:] + (getJv(i)[0,:]).T * getJomega(i)[1,:]

def getD():
    # inertia matrix
    D = Matrix([[0 for i in range(n)] for j in range(n)])
    for i in range(n):
        D += m[i] * getJv(i).T * getJv(i) + getJomega(i).T * R(0, i) * getTensor(i) * R(0, i).T * getJomega(i) + 2 * (m[i] * get_riici(i))[0] * Jx + 2 * (m[i] * get_riici(i))[1] * Jy + 2 * (m[i] * get_riici(i))[2] * Jz
    return D

def getC(D):
    # Coriolis and Centrifugal matrix
    C = [[[0 for k in range(n)] for j in range(n)] for i in range(n)]
    for i in range(n):
        for j in range(n):
            for k in range(n):
                C[i][j][k] = (1/2) * (diff(D[k,j], q[i]) + diff(D[k,i], q[j]) - diff(D[i,j], q[k]))

    C_new = [[0 for k in range(n)] for j in range(n)]
    for k in range(n):
        for j in range(n):
            s = 0
            for i in range(n):
                s += C[i][j][k] * q[i]
            C_new[k][j] = s
    return C_new

def getG(U):
    """ gravity matrix"""
    G = [0 for j in range(n)]
    for j in range(n):
        G[j] = diff(U, q[j])
    return G
