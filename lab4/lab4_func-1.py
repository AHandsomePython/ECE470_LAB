#!/usr/bin/env python

'''

lab4pkg_fk/lab4_func.py

@brief: functions for computing forward kinematics using Product of Exponential (PoE) method
@author: Songjie Xiao
@date: Monday 2023/3/20

'''

import numpy as np
import math
from scipy.linalg import expm

PI = np.pi

"""
You may write some helper functions as you need
Use 'expm' for matrix exponential
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	S = np.zeros((6,6))

	l = [0.152, 0.12, 0.244, 0.093, 0.213, 0.104, 0.085, 0.092, 0.07]
	
	M = np.array([[0, 0, 1, l[1]-l[3]+l[5]+l[7]+l[8]],
			   	  [0, 1, 0, -(l[2]+l[4]+l[6])],
				  [-1, 0, 0, l[0]],
				  [0, 0, 0, 1]])
	w = np.array([[0, 0, 1],
			      [1, 0, 0],
				  [1, 0, 0],
				  [1, 0, 0],
				  [0, -1, 0],
				  [1, 0, 0]]
				  )
	#print(M)
	q = np.array([[0, 0, 0],
			     [l[1], 0, l[0]],
				 [l[1], -l[2], l[0]],
				 [l[1]-l[3], -l[2]-l[4], l[0]],
				 [l[1]-l[3]+l[5], -l[2]-l[4], l[0]],
				 [l[1]-l[3]+l[5], -l[2]-l[4]-l[6], l[0]]]
				 )
	print("q:")
	print(q)
	v = np.array([-np.cross(w[0].T, q[0]), -np.cross(w[1].T, q[1]),-np.cross(w[2].T, q[2]), -np.cross(w[3].T, q[3]), -np.cross(w[4].T, q[4]), -np.cross(w[5].T, q[5])])
	print("v:")
	print(v)
	S = np.append(w, v, axis=1)


	# ==============================================================#


	def transform(x, y):

		trans = np.array([[0,-x[2], x[1], y[0]], [x[2], 0, -x[0], y[1]], [-x[1], x[0], 0, y[2]], [0,0,0,0]])
		return trans

	print(w[0])
	S0 = transform(w[0], v[0])
	S1 = transform(w[1], v[1])
	S2 = transform(w[2], v[2])
	S3 = transform(w[3], v[3])
	S4 = transform(w[4], v[4])
	S5 = transform(w[5], v[5])
	
	print(v[3])
	print("S")
	print(S0)
	print(S1)
	print(S2)
	print(S3)
	print(S4)
	print(S5)
	return M, S0, S1, S2, S3, S4, S5

	"""
	Function that calculates encoder numbers for each motor
	"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = np.eye(4)

	M, S0, S1, S2, S3, S4, S5 = Get_MS()
	#print(expm(np.array([S[0]]).T*theta1))
	print("M:")
	print(M)
	theta1 = theta1-PI/2
	theta4 = theta4+PI/2
	T1 = np.dot(expm(S0*theta1),expm(S1*theta2))
	T2 = np.dot(T1,expm(S2*theta3))
	T3 = np.dot(T2,expm(S3*theta4))
	T5 = np.dot(T3,expm(S4*theta5))
	T6 = np.dot(T5,expm(S5*theta6))
	T = np.dot(T6,M)
	# T=np.exp(S[0].T*theta1)*np.exp(S[1].T*theta2)*np.exp(S[2].T*theta3)*np.exp(S[3].T*theta4)*np.exp(S[4].T*theta5)*np.exp(S[5].T*theta6)






	# ==============================================================#
	print("T:")
	print(str(T) + "\n")
	return T



