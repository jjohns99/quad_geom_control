import numpy as np
from utils import *

#initial conditions
x0 = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
euler0 = np.array([[0.0*np.pi/180], [0.0*np.pi/180], [0.0*np.pi/180]]).T
x0[6:10] = Euler2Quat(euler0)

#dynamic params
m = 1.0
g = 9.81

J = np.diag([0.07, 0.07, 0.12])
D = np.diag([0.1, 0.1, 0])
A = np.diag([0, 0, 0.005]) 

kf = g
kt = 5.0
l = 0.25

input_stdev = 0.1

#quad mixer
mixer = np.array([[kf, kf, kf, kf],
                [0, 0, l*kf, -l*kf],
                [l*kf, -l*kf, 0, 0],
                [-kt, -kt, kt, kt]])

#rotors 3 and 4 clipped?
clipped_props = False

#sim params
ts = 0.01
tf = 60.0

#geometric control params
Kp = np.diag([2.0, 2.0, 2.2])
Kv = np.diag([2.5, 2.5, 2.5])
# kr = 5.0*2
kr = np.diag([10., 10., 10.])
Kom = 0.6*np.diag([1.0, 1.0, 1.0])*2
sig = 0.05

#linear control params
At = np.block([[np.zeros((3,3)), np.eye(3)], [np.zeros((3,3)), np.zeros((3,3))]])
Bt = np.vstack([np.zeros((3,3)), -1.0/m*np.eye(3)])
Qt = np.diag([2.0, 2.0, 2.0, 1.0, 1.0, 1.0])
Rt = np.diag([0.1, 0.1, 1.0])
Qr = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])*0.01
Rr = np.diag([0.05, 0.05, 0.05])

#sinusoid trajectory params
#fast circle
# period = np.array([[2.5, 2.5, 2.5/4]]).T 
# amplitude = np.array([[5.0, 5.0, 0.0]]).T
# phase = np.array([[0.0, np.pi/2.0, 0.0]]).T
# offset = np.array([[0.0, 0.0, -5.0]]).T
#vertical loops
period = np.array([[4.0, 4.0, 4.0]]).T
amplitude = np.array([[0.0, 3.0, 8.0]]).T
phase = np.array([[0.0, 0.0, np.pi/2.0]]).T
offset = np.array([[0.0, 0.0, -8.0]]).T

#bspline trajectory params
#flip trajectory
# knots = np.array([0,0,0,0,1.3,1.7,3,3,3,3])*1.5
# coeffs = np.array([[0,0],[0,0],[6.0,-15],[-4.0,-15],[3.0,0],[3.0,0]]).T 
# degree = 3
#line trajectory
# knots = np.array([0,0,0,0,3,3,3,3])*5
# coeffs = np.array([[0,0],[0,0],[5.0,-3.0],[5.0,-3.0]]).T 
# degree = 3
knots = np.array([0,0,0,0,1.0,2.0,3,3,3,3])*1.5
coeffs = np.array([[0,0],[0,0],[3.0,-8],[6.0,-8],[9.0,0],[9.0,0]]).T 
degree = 3