
import numpy as np
import math
from numpy.linalg import inv
import serial


# pixel size is 5.2um

conv = (5.2*10**(-3))
depth = 1492

def matrix():

	a00 = -(1/ conv)*3.1 
	a01 = 0
	a02 = 320
	a03 = 0


	a10 = 0
	a11 = -(1/ conv)*3.1 
	a12 = 240
	a13 = 0


	a20 = 0
	a21 = 0
	a22 = 1
	a23 = 0


	a30 = 0
	a31 = 0
	a32 = 0
	a33 = 1



	camera_matrix = np.array([[a00,a01,a02, a03] , [a10,a11 ,a12 , a13] , [a20,a21,a22 , a23] , [a30, a31, a32,a33]   ])
	print(camera_matrix)

	return camera_matrix



def rot_trans():

	b00 = 0.978
	b01 = -0.2079
	b02 = 0
	b03 = 9.78


	b10 = 0.2079
	b11 = 0.978
	b12 = 0
	b13 = -2.08

	b20 = 0
	b21 = 0
	b22 = 1
	b23 = 0

	b30 = 0
	b31 = 0
	b32 =0
	b33 = 1

	rot_trans_matrix  = np.array([[b00,b01,b02 , b03] , [b10,b11 ,b12 , b13] , [b20,b21,b22 , b23] ,   [b30, b31, b32,b33]  ])


	print(rot_trans_matrix)

	return rot_trans_matrix



in1 = matrix()
in2 = rot_trans()

m = np.matmul(in1,in2)

camera_matrix_inv = inv(m)

print(camera_matrix_inv)
arr = np.array([[-2] , [192] , [1] , [1/depth] ])



res = np.matmul(camera_matrix_inv, arr)

print(res)
print(depth*res)