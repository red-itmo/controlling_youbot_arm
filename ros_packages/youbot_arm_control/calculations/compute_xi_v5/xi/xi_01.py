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
		opL_0 = -(-4*a[0]**2*sin(q[0] - 0.938888888888889*pi)**4*ddq[0] - 8*a[0]**2*sin(q[0] - 0.938888888888889*pi)**2*cos(q[0] - 0.938888888888889*pi)**2*ddq[0] - 4*a[0]**2*cos(q[0] - 0.938888888888889*pi)**4*ddq[0] + 8*a[0]*a[1]*sin(q[0] - 0.938888888888889*pi)**4*sin(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] - 8*a[0]*a[1]*sin(q[0] - 0.938888888888889*pi)**4*cos(q[1] - 0.861111111111111*pi)*ddq[0] + 16*a[0]*a[1]*sin(q[0] - 0.938888888888889*pi)**2*sin(q[1] - 0.861111111111111*pi)*cos(q[0] - 0.938888888888889*pi)**2*dq[0]*dq[1] - 16*a[0]*a[1]*sin(q[0] - 0.938888888888889*pi)**2*cos(q[0] - 0.938888888888889*pi)**2*cos(q[1] - 0.861111111111111*pi)*ddq[0] + 8*a[0]*a[1]*sin(q[1] - 0.861111111111111*pi)*cos(q[0] - 0.938888888888889*pi)**4*dq[0]*dq[1] - 8*a[0]*a[1]*cos(q[0] - 0.938888888888889*pi)**4*cos(q[1] - 0.861111111111111*pi)*ddq[0] + 8*a[1]**2*sin(q[0] - 0.938888888888889*pi)**4*sin(q[1] - 0.861111111111111*pi)*cos(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] - 4*a[1]**2*sin(q[0] - 0.938888888888889*pi)**4*cos(q[1] - 0.861111111111111*pi)**2*ddq[0] + 16*a[1]**2*sin(q[0] - 0.938888888888889*pi)**2*sin(q[1] - 0.861111111111111*pi)*cos(q[0] - 0.938888888888889*pi)**2*cos(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] - 8*a[1]**2*sin(q[0] - 0.938888888888889*pi)**2*cos(q[0] - 0.938888888888889*pi)**2*cos(q[1] - 0.861111111111111*pi)**2*ddq[0] + a[1]**2*sin(2*q[1] - 1.72222222222222*pi)*cos(2*q[0] - 1.87777777777778*pi)**2*dq[0]*dq[1] + 2*a[1]**2*sin(2*q[1] - 1.72222222222222*pi)*cos(2*q[0] - 1.87777777777778*pi)*dq[0]*dq[1] + a[1]**2*sin(2*q[1] - 1.72222222222222*pi)*dq[0]*dq[1] - 4*a[1]**2*cos(q[0] - 0.938888888888889*pi)**4*cos(q[1] - 0.861111111111111*pi)**2*ddq[0])/4
		return opL_0

	def opL1(self):
		"""101"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_1 = -2*(a[0]*sin(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] - a[0]*cos(q[1] - 0.861111111111111*pi)*ddq[0] + 2*a[1]*sin(q[1] - 0.861111111111111*pi)*cos(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] - a[1]*sin(0.861111111111111*pi)**2*sin(q[1])**2*ddq[0] - 2*a[1]*sin(0.861111111111111*pi)*sin(q[1])*cos(0.861111111111111*pi)*cos(q[1])*ddq[0] - a[1]*cos(0.861111111111111*pi)**2*cos(q[1])**2*ddq[0])
		return opL_1

	def opL2(self):
		"""102"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_2 = 2*(a[0]*sin(q[1] - 0.861111111111111*pi)*ddq[0] + a[0]*cos(q[1] - 0.861111111111111*pi)*dq[0]*dq[1] + a[1]*sin(q[1] - 0.861111111111111*pi)*cos(q[1] - 0.861111111111111*pi)*ddq[0] + 2*a[1]*sin(0.861111111111111*pi)**2*sin(q[1])**2*dq[0]*dq[1] + 4*a[1]*sin(0.861111111111111*pi)*sin(q[1])*cos(0.861111111111111*pi)*cos(q[1])*dq[0]*dq[1] + 2*a[1]*cos(0.861111111111111*pi)**2*cos(q[1])**2*dq[0]*dq[1] - a[1]*dq[0]*dq[1])
		return opL_2

	def opL3(self):
		"""103"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_3 = a[1]*(sin(q[1] - 0.861111111111111*pi)*ddq[1] + cos(q[1] - 0.861111111111111*pi)*dq[1]**2)
		return opL_3

	def opL4(self):
		"""104"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_4 = sin(q[1] - 0.861111111111111*pi)**2*ddq[0] + sin(2*q[1] - 1.72222222222222*pi)*dq[0]*dq[1]
		return opL_4

	def opL5(self):
		"""105"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_5 = -sin(2*q[1] - 1.72222222222222*pi)*dq[0]*dq[1] + cos(q[1] - 0.861111111111111*pi)**2*ddq[0]
		return opL_5

	def opL6(self):
		"""106"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_6 = 0
		return opL_6

	def opL7(self):
		"""107"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_7 = 2*sin(q[1] - 0.861111111111111*pi)**2*dq[0]*dq[1] - sin(2*q[1] - 1.72222222222222*pi)*ddq[0] - 2*cos(q[1] - 0.861111111111111*pi)**2*dq[0]*dq[1]
		return opL_7

	def opL8(self):
		"""108"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_8 = -sin(q[1] - 0.861111111111111*pi)*ddq[1] - cos(q[1] - 0.861111111111111*pi)*dq[1]**2
		return opL_8

	def opL9(self):
		"""109"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_9 = -sin(q[1] - 0.861111111111111*pi)*dq[1]**2 + cos(q[1] - 0.861111111111111*pi)*ddq[1]
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

