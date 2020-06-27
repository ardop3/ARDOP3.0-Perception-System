import numpy as np
import math
from numpy.linalg import inv

def H12(theta1,alpha1):

    K1= theta1 +P

    h12_00 =  math.cos(K1)
    h12_01 = -math.sin(K1)*math.cos(P)
    h12_02 = math.sin(K1)*math.sin(P)
    h12_03 = 0

    h12_10 = math.sin(K1)
    h12_11 = math.cos(K1)*math.cos(P)
    h12_12 = -math.cos(K1)*math.sin(P)
    h12_13 = 0

    h12_20 = 0
    h12_21 = math.sin(P)
    h12_22 = math.cos(P)
    h12_23 = a1

    h12_30 =0
    h12_31 =0
    h12_32 = 0
    h12_33 = 1

    H12 = np.array([[h12_00,h12_01,h12_02,h12_03],[h12_10,h12_11,h12_12,h12_13],[h12_20,h12_21,h12_22,h12_23],[h12_30,h12_31,h12_32,h12_33]])
    #print(H12)

    return H12



def H23(theta2):

    K2= theta2  ##+P

    h23_00 = math.cos(K2)
    h23_01 = -math.sin(K2)*math.cos(0)
    h23_02 = math.sin(K2)*math.sin(0)
    h23_03 = a3*math.cos(K2)

    h23_10 = math.sin(K2)
    h23_11 = math.cos(K2)*math.cos(0)
    h23_12 = -math.cos(K2)*math.sin(0)
    h23_13 =    a3*math.sin(K2)

    h23_20 = 0
    h23_21 = math.sin(0)
    h23_22 = math.cos(0)
    h23_23 = a2

    h23_30  = 0
    h23_31 = 0
    h23_32 = 0
    h23_33 = 1

    H23 = np.array([[h23_00,h23_01,h23_02,h23_03],[h23_10,h23_11,h23_12,h23_13],[h23_20,h23_21,h23_22,h23_23],[h23_30,h23_31,h23_32,h23_33]])

    return H23


pi = 3.14159265359

P = pi/2

a1 = 44
a2 = 0
a3 = 0

theta1 = 0
theta2 = (pi/180)*70


#z_error = 7

xc =  -83.7
yc = 8.67
zc = 12.7

cam_co = np.array([[xc],[yc],[zc],[1]])

tf12 = H12(theta1,theta2)
tf23 = H23(theta2)

tf13 = np.matmul(tf12,tf23)

tf_cam_base = np.matmul(tf13,cam_co)

print(tf_cam_base)
