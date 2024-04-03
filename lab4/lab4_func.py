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
	M = np.eye(4)
	S = np.zeros((6,6))

	M = np.array([
		[0.0, 0.0, 1.0, 293.0],
		[0.0, 1.0, 0.0, -542.0],
		[-1.0, 0.0, 0.0, 152.0],
		[0.0, 0.0, 0.0, 1.0]
	])
	
	S[0, 0:3] = np.array([0.0, 0.0, 1.0])
	S[0, 3:6] = -np.cross(S[0, 0:3], np.array([0, 0, 0]))

	S[1, 0:3] = np.array([1.0, 0.0, 0.0])
	S[1, 3:6] = -np.cross(S[1, 0:3], np.array([120, 0, 152]))

	S[2, 0:3] = np.array([1.0, 0.0, 0.0])
	S[2, 3:6] = -np.cross(S[2, 0:3], np.array([120, -244, 152]))

	S[3, 0:3] = np.array([1.0, 0.0, 0.0])
	S[3, 3:6] = -np.cross(S[3, 0:3], np.array([120-93, -244-213, 152]))

	S[4, 0:3] = np.array([0.0, -1.0, 0.0])
	S[4, 3:6] = -np.cross(S[4, 0:3], np.array([120-93+104, -244-213, 152]))

	S[5, 0:3] = np.array([1.0, 0.0, 0.0])
	S[5, 3:6] = -np.cross(S[5, 0:3], np.array([120-93+104, -542, 152]))


	S = S.T



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	th = [theta1-0.5*PI, theta2, theta3, theta4+0.5*PI, theta5, theta6]
	e = [expm(np.array([
		[0.0, -S[2, i], S[1, i], S[3, i]],
		[S[2, i], 0.0, -S[0, i], S[4, i]],
		[-S[1, i], S[0, i], 0.0, S[5, i]],
		[0.0, 0.0, 0.0, 0.0],
	]) * th[i]) for i in range(6)]
	T = np.eye(4)
	for x in e:
		T = T @ x
	T = T @ M
	T[0:3, 3] = 1e-3 * T[0:3, 3]
	






	# ==============================================================#
	
	print(str(T) + "\n")
	return T




