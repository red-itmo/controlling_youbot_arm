#!/usr/bin/env python3
from numpy import cos, sin, sqrt, pi


class XI:
	"""XI_32"""

	def __init__(self, q=(0,0,0,0,0), dq=(0,0,0,0,0), ddq=(0,0,0,0,0), a=(0,0,0,0,0), d=(0,0,0,0,0)):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def setData(self, q, dq, ddq, a, d):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def opL0(self):
		"""320"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_0 = 1.0*a[2]*(1.0*a[0]*cos(q[1] + q[2])*dq[0]**2 - 0.5*a[1]*sin(2*q[1] + q[2])*dq[0]**2 + 0.5*a[1]*sin(q[2])*dq[0]**2 + 1.0*a[1]*sin(q[2])*dq[1]**2 + 1.0*a[1]*cos(q[2])*ddq[1] - 0.5*a[2]*sin(2*q[1] + 2*q[2])*dq[0]**2 + 1.0*a[2]*ddq[1] + 1.0*a[2]*ddq[2] + 2.22044604925031e-16*sin(q[1] - 3*q[2]) - 9.82*sin(q[1] + q[2]) - 2.22044604925031e-16*sin(3*q[1] - q[2]) + 2.22044604925031e-16*sin(3*q[1] + 3*q[2]) - 2.22044604925031e-16*sin(-3*q[1] + q[2] + 2*q[3]) - 2.22044604925031e-16*sin(q[1] - 3*q[2] + 2*q[3]) + 2.22044604925031e-16*sin(q[1] + q[2] + 2*q[3]) - 2.22044604925031e-16*sin(3*q[1] + 3*q[2] - 2*q[3]))
		return opL_0

	def opL1(self):
		"""321"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_1 = 1.0*(1.0*a[0]*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*sin(q[2] + q[3])*dq[1]**2 - 1.0*a[1]*sin(q[1])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*cos(q[2] + q[3])*ddq[1] + 4.0*a[2]*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*dq[0]**2 - 2.0*a[2]*sin(q[1])*cos(q[1] + q[3])*dq[0]**2 - 2.0*a[2]*sin(q[2])*cos(q[2] + q[3])*dq[0]**2 - 1.0*a[2]*sin(q[3])*dq[0]**2 - 2.0*a[2]*sin(q[3])*dq[1]*dq[3] - 2.0*a[2]*sin(q[3])*dq[2]*dq[3] - 1.0*a[2]*sin(q[3])*dq[3]**2 + 2.0*a[2]*cos(q[3])*ddq[1] + 2.0*a[2]*cos(q[3])*ddq[2] + 1.0*a[2]*cos(q[3])*ddq[3] - 9.82*sin(q[1] + q[2] + q[3]))
		return opL_1

	def opL2(self):
		"""322"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_2 = -a[2]*cos(q[1] + q[2])*ddq[0]
		return opL_2

	def opL3(self):
		"""323"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_3 = 1.0*(1.0*a[0]*sin(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*sin(q[2] + q[3])*ddq[1] - 1.0*a[1]*sin(q[1] + q[2] + q[3])*sin(q[1])*dq[0]**2 - 1.0*a[1]*cos(q[2] + q[3])*dq[1]**2 - 2.0*a[2]*sin(q[1] + q[3])*sin(q[1])*dq[0]**2 - 2.0*a[2]*sin(q[2] + q[3])*sin(q[2])*dq[0]**2 - 4.0*a[2]*sin(q[1])*sin(q[2])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 2.0*a[2]*sin(q[3])*ddq[1] + 2.0*a[2]*sin(q[3])*ddq[2] + 1.0*a[2]*sin(q[3])*ddq[3] + 1.0*a[2]*cos(q[3])*dq[0]**2 + 2.0*a[2]*cos(q[3])*dq[1]*dq[3] + 2.0*a[2]*cos(q[3])*dq[2]*dq[3] + 1.0*a[2]*cos(q[3])*dq[3]**2 + 9.82*cos(q[1] + q[2] + q[3]))
		return opL_3

	def opL4(self):
		"""324"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_4 = sin(2*q[1] + 2*q[2] + 2*q[3])*dq[0]**2/2
		return opL_4

	def opL5(self):
		"""325"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_5 = ddq[1] + ddq[2] + ddq[3]
		return opL_5

	def opL6(self):
		"""326"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_6 = -sin(2*q[1] + 2*q[2] + 2*q[3])*dq[0]**2/2
		return opL_6

	def opL7(self):
		"""327"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_7 = cos(q[1] + q[2] + q[3])*ddq[0]
		return opL_7

	def opL8(self):
		"""328"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_8 = -(-sqrt(2)*sin(q[1] + q[2])*cos(q[3]) - 2*sin(q[2] + pi/4)*sin(q[3])*cos(q[1]) - 2*sin(q[1])*sin(q[2])*cos(q[3] + pi/4) + sqrt(2)*cos(q[1] + q[3])*cos(q[2]))*sin(q[1] + q[2] + q[3] + pi/4)*dq[0]**2
		return opL_8

	def opL9(self):
		"""329"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_9 = sin(q[1] + q[2] + q[3])*ddq[0]
		return opL_9

	def getXI(self, q, dq, ddq):
		self.q = q
		self.dq = dq
		self.ddq = ddq
		XI = [0 for i in range(10)]
		XI[0] = self.opL0()
		XI[1] = self.opL1()
		XI[2] = self.opL2()
		XI[3] = self.opL3()
		XI[4] = self.opL4()
		XI[5] = self.opL5()
		XI[6] = self.opL6()
		XI[7] = self.opL7()
		XI[8] = self.opL8()
		XI[9] = self.opL9()
		return XI

