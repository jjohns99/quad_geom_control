import numpy as np
import so3

def inverse(T):
    R = T[:3,:3]
    t = T[:3,[3]]
    return np.block([[R.T, -R.T @ t], [np.zeros((1,3)), 1.0]])

def hat(tau):
    return np.block([[so3.hat(tau[3:]), tau[:3]], [np.zeros((1,3)), 0.0]])

def vee(mat4):
    print(mat4)
    return np.vstack([mat4[:3,[3]], so3.vee(mat4[:3,:3])])

def Log(T):
    theta = so3.Log(T[:3,:3])
    return np.vstack([so3.J_l_inv(theta) @ T[:3,[3]], theta])

def Q(tau):
    rhox = so3.hat(tau[:3])
    theta = tau[3:]
    th = np.linalg.norm(theta)
    thetax = so3.hat(theta)

    if th < 1e-5:
        return 0.5*rhox

    term1 = (th - np.sin(th))/th**3 * (thetax@rhox + rhox@thetax + thetax@rhox@thetax)
    term2 = (1 - th**2/2 - np.cos(th))/th**4 * (thetax@thetax@rhox + rhox@thetax@thetax - 3*thetax@rhox@thetax)
    term3 = 0.5*((1 - th**2/2 - np.cos(th))/th**4 - 3*(th - np.sin(th) - th**3/6)/th**5) * (thetax@rhox@thetax@thetax + thetax@thetax@rhox@thetax)
    return 0.5*rhox + term1 - term2 - term3


def J_l(tau):
    theta = tau[3:]
    return np.block([[so3.J_l(theta), Q(tau)], [np.zeros((3,3)), so3.J_l(theta)]])

def J_r(tau):
    return J_l(-tau)

def J_l_inv(tau):
    theta = tau[3:]
    j_l_inv = so3.J_l_inv(theta)
    return np.block([[j_l_inv, -j_l_inv@Q(tau)@j_l_inv], [np.zeros((3,3)), j_l_inv]])

def J_r_inv(tau):
    return J_l_inv(-tau)