#!/usr/bin/env python3
from numpy import cos, sin, sqrt, pi


class XI:
	"""XI_43"""

	def __init__(self, q=(0,0,0,0,0), dq=(0,0,0,0,0), ddq=(0,0,0,0,0), a=(0,0,0,0,0), d=(0,0,0,0,0)):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def setData(self, q, dq, ddq, a, d):
		self.q, self.dq, self.ddq = q, dq, ddq
		self.a, self.d = a, d

	def opL0(self):
		"""430"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_0 = 1.0*(1.0*a[0]*d[4]*sin(q[1] + q[2] + q[3])*dq[0]**2 - 7.105427357601e-15*a[0]*sin(q[1])**2*sin(q[2])**2*sin(q[3])**2 - 1.0*a[1]*d[4]*sin(q[1] + q[2] + q[3])*sin(q[1])*dq[0]**2 - 1.0*a[1]*d[4]*sin(q[2])**2*sin(q[3])**2*cos(q[2])*cos(q[3])*dq[1]**2 + 1.0*a[1]*d[4]*sin(q[2])**2*sin(q[3])*cos(q[2])*ddq[1] - 1.0*a[1]*d[4]*sin(q[2])**2*cos(q[2])*cos(q[3])**3*dq[1]**2 + 1.0*a[1]*d[4]*sin(q[2])*sin(q[3])**2*cos(q[3])*ddq[1] + 1.0*a[1]*d[4]*sin(q[2])*sin(q[3])*dq[1]**2 + 1.0*a[1]*d[4]*sin(q[2])*cos(q[3])**3*ddq[1] - 1.0*a[1]*d[4]*sin(q[3])**2*cos(q[2])**3*cos(q[3])*dq[1]**2 + 1.0*a[1]*d[4]*sin(q[3])*cos(q[2])**3*ddq[1] - 1.0*a[1]*d[4]*cos(q[2])**3*cos(q[3])**3*dq[1]**2 + 7.105427357601e-15*a[1]*sin(q[1])*sin(q[2])**2*sin(q[3])**2 - 1.0*a[2]*d[4]*sin(q[1] + q[3])*sin(q[1])*dq[0]**2 - 1.0*a[2]*d[4]*sin(q[2] + q[3])*sin(q[2])*dq[0]**2 - 2.0*a[2]*d[4]*sin(q[1])*sin(q[2])*cos(q[1] + q[2] + q[3])*dq[0]**2 - 1.0*a[2]*d[4]*sin(q[3])**2*cos(q[3])*dq[1]**2 - 2.0*a[2]*d[4]*sin(q[3])**2*cos(q[3])*dq[1]*dq[2] - 1.0*a[2]*d[4]*sin(q[3])**2*cos(q[3])*dq[2]**2 + 1.0*a[2]*d[4]*sin(q[3])*ddq[1] + 1.0*a[2]*d[4]*sin(q[3])*ddq[2] - 1.0*a[2]*d[4]*cos(q[3])**3*dq[1]**2 - 2.0*a[2]*d[4]*cos(q[3])**3*dq[1]*dq[2] - 1.0*a[2]*d[4]*cos(q[3])**3*dq[2]**2 - 1.4210854715202e-14*a[2]*sin(q[1] + q[2])*sin(q[1])**2*sin(q[2])**2*sin(q[3])**2 - 2.0*d[4]**2*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*dq[0]**2 - 2.0*d[4]**2*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*dq[0]**2 - 2.0*d[4]**2*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*dq[0]**2 - 4.0*d[4]**2*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*d[4]**2*sin(q[1])*cos(q[1])*dq[0]**2 + 1.0*d[4]**2*sin(q[2])*cos(q[2])*dq[0]**2 + 1.0*d[4]**2*sin(q[3])*cos(q[3])*dq[0]**2 + 1.0*d[4]**2*ddq[1] + 1.0*d[4]**2*ddq[2] + 1.0*d[4]**2*ddq[3] + 2.8421709430404e-14*d[4]*sin(q[1] + q[2])*sin(q[1])**2*sin(q[2])**2*sin(q[3])**3 - 2.8421709430404e-14*d[4]*sin(q[1] + q[2])*sin(q[1])**2*sin(q[2])**2*sin(q[3]) - 2.8421709430404e-14*d[4]*sin(q[1] + q[3])*sin(q[1])**2*sin(q[2])*sin(q[3])**2 - 2.8421709430404e-14*d[4]*sin(q[2] + q[3])*sin(q[1])*sin(q[2])**2*sin(q[3])**2 + 2.8421709430404e-14*d[4]*sin(q[1])**3*sin(q[2])**3*sin(q[3])**2*cos(q[3]) + 1.4210854715202e-14*d[4]*sin(q[1])**3*sin(q[2])*cos(q[3])**3 + 1.4210854715202e-14*d[4]*sin(q[1])**3*sin(q[3])*cos(q[2])**3 + 9.81999999999996*d[4]*sin(q[1])**2*sin(q[2])**2*sin(q[3])**2*cos(q[1])*cos(q[2])*cos(q[3]) + 9.82000000000001*d[4]*sin(q[1])**2*sin(q[2])**2*cos(q[1])*cos(q[2])*cos(q[3])**3 - 9.81999999999996*d[4]*sin(q[1])**2*sin(q[2])*sin(q[3])*cos(q[1]) + 9.82000000000001*d[4]*sin(q[1])**2*sin(q[3])**2*cos(q[1])*cos(q[2])**3*cos(q[3]) + 9.82*d[4]*sin(q[1])**2*cos(q[1])*cos(q[2])**3*cos(q[3])**3 + 1.4210854715202e-14*d[4]*sin(q[1])*sin(q[2])**3*cos(q[3])**3 - 9.81999999999996*d[4]*sin(q[1])*sin(q[2])**2*sin(q[3])*cos(q[2]) - 9.81999999999996*d[4]*sin(q[1])*sin(q[2])*sin(q[3])**2*cos(q[3]) - 9.82000000000001*d[4]*sin(q[1])*sin(q[2])*cos(q[3])**3 + 1.4210854715202e-14*d[4]*sin(q[1])*sin(q[3])**3*cos(q[2])**3 - 9.82000000000001*d[4]*sin(q[1])*sin(q[3])*cos(q[2])**3 + 1.4210854715202e-14*d[4]*sin(q[2])**3*sin(q[3])*cos(q[1])**3 + 9.82000000000001*d[4]*sin(q[2])**2*sin(q[3])**2*cos(q[1])**3*cos(q[2])*cos(q[3]) + 9.82*d[4]*sin(q[2])**2*cos(q[1])**3*cos(q[2])*cos(q[3])**3 + 1.4210854715202e-14*d[4]*sin(q[2])*sin(q[3])**3*cos(q[1])**3 - 9.82000000000001*d[4]*sin(q[2])*sin(q[3])*cos(q[1])**3 + 9.82*d[4]*sin(q[3])**2*cos(q[1])**3*cos(q[2])**3*cos(q[3]) + 9.82*d[4]*cos(q[1])**3*cos(q[2])**3*cos(q[3])**3)
		return opL_0

	def opL1(self):
		"""431"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_1 = 1.0*(1.0*a[0]*cos(q[1] + q[2] + q[3])*cos(q[4])*dq[0]**2 + 1.0*a[1]*sin(q[2] + q[3])*cos(q[4])*dq[1]**2 - 1.0*a[1]*sin(q[1])*cos(q[1] + q[2] + q[3])*cos(q[4])*dq[0]**2 + 1.0*a[1]*cos(q[2] + q[3])*cos(q[4])*ddq[1] + 2.0*a[2]*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*cos(q[4])*dq[0]**2 - 1.0*a[2]*sin(q[1])*cos(q[1] + q[3])*cos(q[4])*dq[0]**2 - 1.0*a[2]*sin(q[2])*cos(q[2] + q[3])*cos(q[4])*dq[0]**2 + 1.0*a[2]*sin(q[3])*cos(q[4])*dq[1]**2 + 2.0*a[2]*sin(q[3])*cos(q[4])*dq[1]*dq[2] + 1.0*a[2]*sin(q[3])*cos(q[4])*dq[2]**2 + 1.0*a[2]*cos(q[3])*cos(q[4])*ddq[1] + 1.0*a[2]*cos(q[3])*cos(q[4])*ddq[2] + 8.0*d[4]*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4])*dq[0]**2 - 2.0*d[4]*sin(q[1])**2*cos(q[4])*dq[0]**2 + 1.0*d[4]*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*ddq[0] + 2.0*d[4]*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4])*dq[0]*dq[4] - 4.0*d[4]*sin(q[1])*sin(q[2])*cos(q[1] + q[2])*cos(q[4])*dq[0]**2 - 4.0*d[4]*sin(q[1])*sin(q[3])*cos(q[1] + q[3])*cos(q[4])*dq[0]**2 - 1.0*d[4]*sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3])*ddq[0] - 2.0*d[4]*sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[0]*dq[4] - 2.0*d[4]*sin(q[2])**2*cos(q[4])*dq[0]**2 - 4.0*d[4]*sin(q[2])*sin(q[3])*cos(q[2] + q[3])*cos(q[4])*dq[0]**2 - 1.0*d[4]*sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3])*ddq[0] - 2.0*d[4]*sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4])*dq[0]*dq[4] - 2.0*d[4]*sin(q[3])**2*cos(q[4])*dq[0]**2 - 1.0*d[4]*sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2])*ddq[0] - 2.0*d[4]*sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4])*dq[0]*dq[4] - 1.0*d[4]*sin(q[4])*ddq[4] + 1.0*d[4]*cos(q[4])*dq[0]**2 - 1.0*d[4]*cos(q[4])*dq[4]**2 - 9.82*sin(q[1] + q[2] + q[3])*cos(q[4]))
		return opL_1

	def opL2(self):
		"""432"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_2 = -1.0*(1.0*a[0]*sin(q[4])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*sin(q[2] + q[3])*sin(q[4])*dq[1]**2 - 1.0*a[1]*sin(q[1])*sin(q[4])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*sin(q[4])*cos(q[2] + q[3])*ddq[1] + 2.0*a[2]*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*sin(q[4])*dq[0]**2 - 1.0*a[2]*sin(q[1])*sin(q[4])*cos(q[1] + q[3])*dq[0]**2 - 1.0*a[2]*sin(q[2])*sin(q[4])*cos(q[2] + q[3])*dq[0]**2 + 1.0*a[2]*sin(q[3])*sin(q[4])*dq[1]**2 + 2.0*a[2]*sin(q[3])*sin(q[4])*dq[1]*dq[2] + 1.0*a[2]*sin(q[3])*sin(q[4])*dq[2]**2 + 1.0*a[2]*sin(q[4])*cos(q[3])*ddq[1] + 1.0*a[2]*sin(q[4])*cos(q[3])*ddq[2] + 8.0*d[4]*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[0]**2 - 2.0*d[4]*sin(q[1])**2*sin(q[4])*dq[0]**2 + 2.0*d[4]*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[0]*dq[4] - 1.0*d[4]*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4])*ddq[0] - 4.0*d[4]*sin(q[1])*sin(q[2])*sin(q[4])*cos(q[1] + q[2])*dq[0]**2 - 4.0*d[4]*sin(q[1])*sin(q[3])*sin(q[4])*cos(q[1] + q[3])*dq[0]**2 - 2.0*d[4]*sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3])*dq[0]*dq[4] + 1.0*d[4]*sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*ddq[0] - 2.0*d[4]*sin(q[2])**2*sin(q[4])*dq[0]**2 - 4.0*d[4]*sin(q[2])*sin(q[3])*sin(q[4])*cos(q[2] + q[3])*dq[0]**2 - 2.0*d[4]*sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3])*dq[0]*dq[4] + 1.0*d[4]*sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4])*ddq[0] - 2.0*d[4]*sin(q[3])**2*sin(q[4])*dq[0]**2 - 2.0*d[4]*sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2])*dq[0]*dq[4] + 1.0*d[4]*sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4])*ddq[0] + 1.0*d[4]*sin(q[4])*dq[0]**2 - 1.0*d[4]*sin(q[4])*dq[4]**2 + 1.0*d[4]*cos(q[4])*ddq[4] - 9.82*sin(q[1] + q[2] + q[3])*sin(q[4]))
		return opL_2

	def opL3(self):
		"""433"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_3 = 1.0*(1.0*a[0]*sin(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[1]*sin(q[2] + q[3])*ddq[1] - 1.0*a[1]*sin(q[1] + q[2] + q[3])*sin(q[1])*dq[0]**2 - 1.0*a[1]*cos(q[2] + q[3])*dq[1]**2 - 1.0*a[2]*sin(q[1] + q[3])*sin(q[1])*dq[0]**2 - 1.0*a[2]*sin(q[2] + q[3])*sin(q[2])*dq[0]**2 - 2.0*a[2]*sin(q[1])*sin(q[2])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 1.0*a[2]*sin(q[3])*ddq[1] + 1.0*a[2]*sin(q[3])*ddq[2] - 1.0*a[2]*cos(q[3])*dq[1]**2 - 2.0*a[2]*cos(q[3])*dq[1]*dq[2] - 1.0*a[2]*cos(q[3])*dq[2]**2 - 4.0*d[4]*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*dq[0]**2 - 4.0*d[4]*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*dq[0]**2 - 4.0*d[4]*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*dq[0]**2 - 8.0*d[4]*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[1] + q[2] + q[3])*dq[0]**2 + 2.0*d[4]*sin(q[1])*cos(q[1])*dq[0]**2 + 2.0*d[4]*sin(q[2])*cos(q[2])*dq[0]**2 + 2.0*d[4]*sin(q[3])*cos(q[3])*dq[0]**2 + 2.0*d[4]*ddq[1] + 2.0*d[4]*ddq[2] + 2.0*d[4]*ddq[3] + 9.82*cos(q[1] + q[2] + q[3]))
		return opL_3

	def opL4(self):
		"""434"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_4 = -2*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*cos(q[4])**2*dq[0]**2 - 2*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*cos(q[4])**2*dq[0]**2 - 2*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*cos(q[4])**2*dq[0]**2 - 4*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[1] + q[2] + q[3])*cos(q[4])**2*dq[0]**2 + sin(q[1])*cos(q[1])*cos(q[4])**2*dq[0]**2 + sin(q[2])*cos(q[2])*cos(q[4])**2*dq[0]**2 + sin(q[3])*cos(q[3])*cos(q[4])**2*dq[0]**2 + sin(q[4])*cos(q[1] + q[2] + q[3])*cos(q[4])*ddq[0] + 2*sin(q[4])*cos(q[4])*dq[1]*dq[4] + 2*sin(q[4])*cos(q[4])*dq[2]*dq[4] + 2*sin(q[4])*cos(q[4])*dq[3]*dq[4] + 2*cos(q[1] + q[2] + q[3])*cos(q[4])**2*dq[0]*dq[4] - cos(q[1] + q[2] + q[3])*dq[0]*dq[4] - cos(q[4])**2*ddq[1] - cos(q[4])**2*ddq[2] - cos(q[4])**2*ddq[3] + ddq[1] + ddq[2] + ddq[3]
		return opL_4

	def opL5(self):
		"""435"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_5 = -2*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*sin(q[4])**2*dq[0]**2 - 2*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*sin(q[4])**2*dq[0]**2 - 2*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*sin(q[4])**2*dq[0]**2 - 4*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])**2*cos(q[1] + q[2] + q[3])*dq[0]**2 + sin(q[1])*sin(q[4])**2*cos(q[1])*dq[0]**2 + sin(q[2])*sin(q[4])**2*cos(q[2])*dq[0]**2 + sin(q[3])*sin(q[4])**2*cos(q[3])*dq[0]**2 + 2*sin(q[4])**2*cos(q[1] + q[2] + q[3])*dq[0]*dq[4] - sin(q[4])**2*ddq[1] - sin(q[4])**2*ddq[2] - sin(q[4])**2*ddq[3] - sin(q[4])*cos(q[1] + q[2] + q[3])*cos(q[4])*ddq[0] - 2*sin(q[4])*cos(q[4])*dq[1]*dq[4] - 2*sin(q[4])*cos(q[4])*dq[2]*dq[4] - 2*sin(q[4])*cos(q[4])*dq[3]*dq[4] - cos(q[1] + q[2] + q[3])*dq[0]*dq[4] + ddq[1] + ddq[2] + ddq[3]
		return opL_5

	def opL6(self):
		"""436"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_6 = (2*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*dq[0] + 2*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*dq[0] + 2*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*dq[0] + 4*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[1] + q[2] + q[3])*dq[0] + sin(q[1])*sin(q[2])*cos(q[3])*dq[4] + sin(q[1])*sin(q[3])*cos(q[2])*dq[4] - sin(q[1])*cos(q[1])*dq[0] + sin(q[2])*sin(q[3])*cos(q[1])*dq[4] - sin(q[2])*cos(q[2])*dq[0] - sin(q[3])*cos(q[3])*dq[0] - cos(q[1])*cos(q[2])*cos(q[3])*dq[4])*dq[0]
		return opL_6

	def opL7(self):
		"""437"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_7 = 4*sin(q[1] + q[2])*sin(q[1])*sin(q[2])*sin(q[4])*cos(q[4])*dq[0]**2 + 4*sin(q[1] + q[3])*sin(q[1])*sin(q[3])*sin(q[4])*cos(q[4])*dq[0]**2 + 4*sin(q[2] + q[3])*sin(q[2])*sin(q[3])*sin(q[4])*cos(q[4])*dq[0]**2 + 8*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*cos(q[1] + q[2] + q[3])*cos(q[4])*dq[0]**2 - 2*sin(q[1])*sin(q[4])*cos(q[1])*cos(q[4])*dq[0]**2 - 2*sin(q[2])*sin(q[4])*cos(q[2])*cos(q[4])*dq[0]**2 - 2*sin(q[3])*sin(q[4])*cos(q[3])*cos(q[4])*dq[0]**2 - 2*sin(q[4])**2*cos(q[1] + q[2] + q[3])*ddq[0] - 4*sin(q[4])**2*dq[1]*dq[4] - 4*sin(q[4])**2*dq[2]*dq[4] - 4*sin(q[4])**2*dq[3]*dq[4] - 4*sin(q[4])*cos(q[1] + q[2] + q[3])*cos(q[4])*dq[0]*dq[4] + 2*sin(q[4])*cos(q[4])*ddq[1] + 2*sin(q[4])*cos(q[4])*ddq[2] + 2*sin(q[4])*cos(q[4])*ddq[3] + cos(q[1] + q[2] + q[3])*ddq[0] + 2*dq[1]*dq[4] + 2*dq[2]*dq[4] + 2*dq[3]*dq[4]
		return opL_7

	def opL8(self):
		"""438"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_8 = sin(q[1] + q[2])*cos(q[3])*cos(q[4])*dq[0]*dq[4] - 8*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4])*dq[0]**2 + sin(q[1] + q[2] + q[3])*sin(q[4])*ddq[0] + 2*sin(q[1])**2*cos(q[4])*dq[0]**2 - 2*sin(q[1])*sin(q[2])*sin(q[3])*cos(q[4])*dq[0]*dq[4] + 4*sin(q[1])*sin(q[2])*cos(q[1] + q[2])*cos(q[4])*dq[0]**2 + 4*sin(q[1])*sin(q[3])*cos(q[1] + q[3])*cos(q[4])*dq[0]**2 + sin(q[1])*cos(q[2])*cos(q[3])*cos(q[4])*dq[0]*dq[4] + 2*sin(q[2])**2*cos(q[4])*dq[0]**2 + 4*sin(q[2])*sin(q[3])*cos(q[2] + q[3])*cos(q[4])*dq[0]**2 + sin(q[2])*cos(q[1])*cos(q[3])*cos(q[4])*dq[0]*dq[4] + 2*sin(q[3])**2*cos(q[4])*dq[0]**2 + 2*sin(q[3])*cos(q[1])*cos(q[2])*cos(q[4])*dq[0]*dq[4] + sin(q[4])*ddq[4] - cos(q[4])*dq[0]**2 + cos(q[4])*dq[4]**2
		return opL_8

	def opL9(self):
		"""439"""
		q, dq, ddq = self.q, self.dq, self.ddq
		a, d = self.a, self.d
		opL_9 = -sin(q[1] + q[2])*sin(q[4])*cos(q[3])*dq[0]*dq[4] + 8*sin(q[1] + q[2] + q[3])*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[0]**2 + sin(q[1] + q[2] + q[3])*cos(q[4])*ddq[0] - 2*sin(q[1])**2*sin(q[4])*dq[0]**2 + 2*sin(q[1])*sin(q[2])*sin(q[3])*sin(q[4])*dq[0]*dq[4] - 4*sin(q[1])*sin(q[2])*sin(q[4])*cos(q[1] + q[2])*dq[0]**2 - 4*sin(q[1])*sin(q[3])*sin(q[4])*cos(q[1] + q[3])*dq[0]**2 - sin(q[1])*sin(q[4])*cos(q[2])*cos(q[3])*dq[0]*dq[4] - 2*sin(q[2])**2*sin(q[4])*dq[0]**2 - 4*sin(q[2])*sin(q[3])*sin(q[4])*cos(q[2] + q[3])*dq[0]**2 - sin(q[2])*sin(q[4])*cos(q[1])*cos(q[3])*dq[0]*dq[4] - 2*sin(q[3])**2*sin(q[4])*dq[0]**2 - 2*sin(q[3])*sin(q[4])*cos(q[1])*cos(q[2])*dq[0]*dq[4] + sin(q[4])*dq[0]**2 - sin(q[4])*dq[4]**2 + cos(q[4])*ddq[4]
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

