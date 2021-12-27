import numpy as np
from so3_py import *

#initial conditions
x0 = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
euler0 = np.array([[180.0*np.pi/180], [0.0*np.pi/180], [0.0*np.pi/180]]).T
x0[6:10] = Euler2Quat(euler0)

#dynamic params
m = 1.0
g = 9.81

J = np.diag([0.07, 0.07, 0.12])
D = np.diag([0.2, 0.2, 0])
A = np.diag([0, 0, 0.005]) 

kf = g
kt = 5.0
l = 0.25

input_stdev = 0.04

#quad mixer
mixer = np.array([[kf, kf, kf, kf],
                [0, 0, l*kf, -l*kf],
                [l*kf, -l*kf, 0, 0],
                [-kt, -kt, kt, kt]])

control_kf = kf*1.1
control_mixer = np.array([[control_kf, control_kf, control_kf, control_kf],
                          [0, 0, l*control_kf, -l*control_kf],
                          [l*control_kf, -l*control_kf, 0, 0],
                          [-kt, -kt, kt, kt]])

#rotors 3 and 4 clipped?
clipped_props = False

#sim params
ts = 0.01
tf = 10.0

#geometric control params
Kp = np.diag([2.0, 2.0, 2.2])
Kv = np.diag([2.5, 2.5, 2.5])
kr_lee1 = 5.0*2
kr_lee2 = 5.0*5
kr = np.diag([10., 10., 10.])
# Kom = 0.6*np.diag([1.0, 1.0, 1.0])*2
Kom = 15*J
sig = 0.05

#lee3 (global hybrid control paper from 2015) parameters
kr1_lee3 = 20.0 # according to the paper, kr1 != kr2. Don't know why.
kr2_lee3 = 15.0
alpha_lee3 = 1.99 # 1 < alpha < 2
beta_lee3 = 0.98 # |beta| < alpha - 1
delta_lee3 = 0.05 # 0 < delta < min(k1, k2)*min(2-alpha, alpha-|beta|-1)
om_lim_lee3 = 500.0
Kom_lee3 = 20*J

control_types = ["ours", "lee1", "lee2", "lee3"]
control_type = control_types[1]

#SE(3) control params
# Kt = np.diag([2.0, 2.0, 2.2, 2., 2., 2.])
# Kv = np.diag([2.5, 2.5, 2.5])
# Kom = 1.2*np.diag([1.0, 1.0, 1.0])

# #linear control params
# At = np.block([[np.zeros((3,3)), np.eye(3)], [np.zeros((3,3)), np.zeros((3,3))]])
# Bt = np.vstack([np.zeros((3,3)), -1.0/m*np.eye(3)])
# Qt = np.diag([2.0, 2.0, 2.0, 1.0, 1.0, 1.0])
# Rt = np.diag([0.1, 0.1, 1.0])
# Qr = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])*0.01
# Rr = np.diag([0.05, 0.05, 0.05])

#linear control params (with integrator)
At = np.block([[np.zeros((3,3)), np.eye(3), np.zeros((3,3))], [np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))], [np.eye(3), np.zeros((3,3)), np.zeros((3,3))]])
Bt = np.vstack([np.zeros((3,3)), -1.0/m*np.eye(3), np.zeros((3,3))])
Qt = np.diag([2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 0.001, 0.001, 0.1])
Rt = np.diag([0.1, 0.1, 1.0])
Qr = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])*0.01
Rr = np.diag([0.05, 0.05, 0.05])

#sinusoid trajectory params
#fast circle$
period = np.array([[2.5, 2.5, 2.5/4]]).T 
amplitude = np.array([[5.0, 5.0, 0.0]]).T
phase = np.array([[0.0, np.pi/2.0, 0.0]]).T
offset = np.array([[0.0, 0.0, -5.0]]).T
#vertical loops
# period = np.array([[4.0, 4.0, 4.0]]).T * 0.35
# amplitude = np.array([[0.0, 2.0, 3.0]]).T * 0.5
# phase = np.array([[0.0, 0.0, np.pi/2.0]]).T
# offset = np.array([[0.0, 0.0, -3.0]]).T * 0.5

#bspline trajectory params
#flip trajectory
# knots = np.array([0, 0, 0, 0.1, 0.2, 0.4, 0.6, 1.2, 1.6, 1.8, 2.2, 2.4, 2.8, 3.4, 3.6, 3.8, 3.9, 4, 4, 4])*1.5
# coeffs = np.array([[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[1.0,-2],[-1.0,-2],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]).T 
# degree = 5
#line trajectory
# knots = np.array([0,0,0,0,3,3,3,3])*5
# coeffs = np.array([[0,0],[0,0],[5.0,-3.0],[5.0,-3.0]]).T 
# degree = 3
# knots = np.array([0,0,0,0,1.0,2.0,3,3,3,3])*1.5
# coeffs = np.array([[0,0],[0,0],[3.0,-8],[6.0,-8],[9.0,0],[9.0,0]]).T 
# degree = 3

# best so far
# knots = np.array([0, 0, 0, 0, 0, 0, 0.75, 1.5, 1.9, 2.1, 2.5, 3.25, 4, 4, 4, 4, 4, 4])*0.65
# coeffs = np.array([[0,0],[0,0],[0,0],[0,0],[0.8,0],[0.8,-2.0],[-1.2,-2.0],[-1.2,0],[0,0],[0,0],[0,0],[0,0]]).T 
# degree = 5

knots = np.array([0, 0, 0, 0, 0, 0, 0.75, 1.5, 1.9, 2.1, 2.5, 3.25, 4, 4, 4, 4, 4, 4])*0.65
coeffs = np.array([[0,0],[0,0],[0,0],[0,0],[0.8,0],[0.8,-2.0],[-1.2,-2.0],[-1.2,0],[-1.2,0],[-1.2,0],[-1.2,0],[-1.2,0]]).T 
degree = 5