#!/usr/bin/env python3
from numpy import cos, sin, sqrt, pi


class XI:
	"""XI_20"""

	def __init__(self, q=(0,0,0,0,0), dq=(0,0,0,0,0), ddq=(0,0,0,0,0), a=(0,0,0,0,0), d=(0,0,0,0,0)):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def setData(self, q, dq, ddq, a, d):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def opL0(self):
		"""200"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_0 = -(-2*a[0]**2*ddq[0] + 4*a[0]*a[1]*sin(q[1])*ddq[0] + 4*a[0]*a[1]*cos(q[1])*dq[0]*dq[1] + 4*a[0]*a[2]*sin(q[1] + q[2])*ddq[0] + 4*a[0]*a[2]*cos(q[1] + q[2])*dq[0]*dq[1] + 4*a[0]*a[2]*cos(q[1] + q[2])*dq[0]*dq[2] - 2*a[1]**2*sin(2*q[1])*dq[0]*dq[1] + a[1]**2*cos(2*q[1])*ddq[0] - a[1]**2*ddq[0] - 4*a[1]*a[2]*sin(2*q[1] + q[2])*dq[0]*dq[1] - 2*a[1]*a[2]*sin(2*q[1] + q[2])*dq[0]*dq[2] + 2*a[1]*a[2]*sin(q[2])*dq[0]*dq[2] + 2*a[1]*a[2]*cos(2*q[1] + q[2])*ddq[0] - 2*a[1]*a[2]*cos(q[2])*ddq[0] - 2*a[2]**2*sin(2*q[1] + 2*q[2])*dq[0]*dq[1] - 2*a[2]**2*sin(2*q[1] + 2*q[2])*dq[0]*dq[2] + a[2]**2*cos(2*q[1] + 2*q[2])*ddq[0] - a[2]**2*ddq[0])/2
		return opL_0

	def opL1(self):
		"""201"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_1 = -2*a[0]*sin(q[1] + q[2])*ddq[0] - 2*a[0]*cos(q[1] + q[2])*dq[0]*dq[1] - 2*a[0]*cos(q[1] + q[2])*dq[0]*dq[2] + 2*a[1]*sin(2*q[1] + q[2])*dq[0]*dq[1] + a[1]*sin(2*q[1] + q[2])*dq[0]*dq[2] - a[1]*sin(q[2])*dq[0]*dq[2] - a[1]*cos(2*q[1] + q[2])*ddq[0] + a[1]*cos(q[2])*ddq[0] + 2*a[2]*sin(2*q[1] + 2*q[2])*dq[0]*dq[1] + 2*a[2]*sin(2*q[1] + 2*q[2])*dq[0]*dq[2] - a[2]*cos(2*q[1] + 2*q[2])*ddq[0] + a[2]*ddq[0]
		return opL_1

	def opL2(self):
		"""202"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_2 = 2*a[0]*sin(q[1] + q[2])*dq[0]*dq[1] + 2*a[0]*sin(q[1] + q[2])*dq[0]*dq[2] - 2*a[0]*cos(q[1] + q[2])*ddq[0] + a[1]*sin(2*q[1] + q[2])*ddq[0] - a[1]*sin(q[2])*ddq[0] + 2*a[1]*cos(2*q[1] + q[2])*dq[0]*dq[1] + a[1]*cos(2*q[1] + q[2])*dq[0]*dq[2] - a[1]*cos(q[2])*dq[0]*dq[2] + a[2]*sin(2*q[1] + 2*q[2])*ddq[0] + 2*a[2]*cos(2*q[1] + 2*q[2])*dq[0]*dq[1] + 2*a[2]*cos(2*q[1] + 2*q[2])*dq[0]*dq[2]
		return opL_2

	def opL3(self):
		"""203"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_3 = a[1]*sin(q[1])*dq[1]**2 - a[1]*cos(q[1])*ddq[1] + a[2]*sin(q[1] + q[2])*dq[1]**2 + 2*a[2]*sin(q[1] + q[2])*dq[1]*dq[2] + a[2]*sin(q[1] + q[2])*dq[2]**2 - a[2]*cos(q[1] + q[2])*ddq[1] - a[2]*cos(q[1] + q[2])*ddq[2]
		return opL_3

	def opL4(self):
		"""204"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_4 = -(2*sin(2*q[1] + 2*q[2])*dq[0]*dq[1] + 2*sin(2*q[1] + 2*q[2])*dq[0]*dq[2] - cos(2*q[1] + 2*q[2])*ddq[0] - ddq[0])/2
		return opL_4

	def opL5(self):
		"""205"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_5 = (2*sin(2*q[1] + 2*q[2])*dq[0]*dq[1] + 2*sin(2*q[1] + 2*q[2])*dq[0]*dq[2] - cos(2*q[1] + 2*q[2])*ddq[0] + ddq[0])/2
		return opL_5

	def opL6(self):
		"""206"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_6 = 0
		return opL_6

	def opL7(self):
		"""207"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_7 = -sin(2*q[1] + 2*q[2])*ddq[0] - 2*cos(2*q[1] + 2*q[2])*dq[0]*dq[1] - 2*cos(2*q[1] + 2*q[2])*dq[0]*dq[2]
		return opL_7

	def opL8(self):
		"""208"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_8 = -sin(q[1] + q[2])*dq[1]**2 - 2*sin(q[1] + q[2])*dq[1]*dq[2] - sin(q[1] + q[2])*dq[2]**2 + cos(q[1] + q[2])*ddq[1] + cos(q[1] + q[2])*ddq[2]
		return opL_8

	def opL9(self):
		"""209"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_9 = -sin(q[1] + q[2])*ddq[1] - sin(q[1] + q[2])*ddq[2] - cos(q[1] + q[2])*dq[1]**2 - 2*cos(q[1] + q[2])*dq[1]*dq[2] - cos(q[1] + q[2])*dq[2]**2
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

