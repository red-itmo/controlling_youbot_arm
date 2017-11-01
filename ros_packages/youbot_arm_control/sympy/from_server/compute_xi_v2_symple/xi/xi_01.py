#!/usr/bin/env python3
from numpy import cos, sin


class XI:
	"""XI_10"""

	def __init__(self, q=(0,0,0,0,0), dq=(0,0,0,0,0), ddq=(0,0,0,0,0), a=(0,0,0,0,0), d=(0,0,0,0,0)):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def setData(self, q, dq, ddq, a, d):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def opL0(self):
		"""100"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_0 = -1.0*(-1.0*a[0]**2*sin(q[0])**6*ddq[0] + 1.0*a[0]**2*sin(q[0])**5*cos(q[0])*dq[0]**2 - 2.0*a[0]**2*sin(q[0])**4*cos(q[0])*ddq[0] + 3.0*a[0]**2*sin(q[0])**4*ddq[0] + 2.0*a[0]**2*sin(q[0])**3*cos(q[0])**3*dq[0]**2 - 2.0*a[0]**2*sin(q[0])**2*ddq[0] + 1.0*a[0]**2*sin(q[0])*cos(q[0])**5*dq[0]**2 + 1.0*a[0]**2*sin(q[0])*dq[0]**2 - 1.0*a[0]**2*cos(q[0])**6*ddq[0] + 2.0*a[0]**2*cos(q[0])**5*ddq[0] - 4.0*a[0]**2*cos(q[0])**3*ddq[0] - 1.0*a[0]**2*ddq[0] + 1.0*a[0]*d[0]*sin(q[0])**7*ddq[1] - 3.0*a[0]*d[0]*sin(q[0])**5*ddq[1] + 3.0*a[0]*d[0]*sin(q[0])**3*ddq[1] + 1.0*a[0]*d[0]*sin(q[0])*cos(q[0])**6*ddq[1] - 9.82*a[0]*cos(q[0]) - 1.0*d[0]**2*sin(q[0])**6*ddq[0] + 3.0*d[0]**2*sin(q[0])**4*ddq[0] - 3.0*d[0]**2*sin(q[0])**2*ddq[0] - 1.0*d[0]**2*cos(q[0])**6*ddq[0])
		return opL_0

	def opL1(self):
		"""101"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_1 = -1.0*(2.0*a[0]*sin(q[0])**3*cos(q[0])*dq[0]**2 + 2.0*a[0]*sin(q[0])*cos(q[0])**3*dq[0]**2 + 1.0*a[0]*sin(q[0])*dq[0]**2 - 2.0*a[0]*cos(q[0])**2*ddq[0] - 2.0*a[0]*cos(q[0])*ddq[0] + 1.0*d[0]*sin(q[0])*ddq[1] - 9.82*cos(q[0]))
		return opL_1

	def opL2(self):
		"""102"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_2 = -a[0]*sin(q[0])*ddq[1] + 2*d[0]*ddq[0]
		return opL_2

	def opL3(self):
		"""103"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_3 = 1.0*(2.0*a[0]*sin(q[0])**3*cos(q[0])*ddq[0] - 2.0*a[0]*sin(q[0])**2*dq[0]**2 + 2.0*a[0]*sin(q[0])*cos(q[0])**3*ddq[0] + 2.0*a[0]*sin(q[0])*ddq[0] + 1.0*a[0]*cos(q[0])*dq[0]**2 + 1.0*a[0]*dq[0]**2 + 1.0*d[0]*sin(q[0])**4*cos(q[0])*ddq[1] - 1.0*d[0]*cos(q[0])**5*ddq[1] + 2.0*d[0]*cos(q[0])**3*ddq[1] + 9.82*sin(q[0]))
		return opL_3

	def opL4(self):
		"""104"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_4 = (2*sin(q[0])**2*ddq[0] + sin(2*q[0])*dq[0]**2)/2
		return opL_4

	def opL5(self):
		"""105"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_5 = 0
		return opL_5

	def opL6(self):
		"""106"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_6 = -(sin(2*q[0])*dq[0]**2 - 2*cos(q[0])**2*ddq[0])/2
		return opL_6

	def opL7(self):
		"""107"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_7 = sin(q[0])*ddq[1]
		return opL_7

	def opL8(self):
		"""108"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_8 = -sin(2*q[0])*ddq[0] - cos(2*q[0])*dq[0]**2
		return opL_8

	def opL9(self):
		"""109"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_9 = -cos(q[0])*ddq[1]
		return opL_9

	def getXI(self, q, dq, ddq, a, d):
		self.setData(q, dq, ddq, a, d)
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

