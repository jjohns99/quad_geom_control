import numpy as np

def QuatTimes(p, q):
    pw = p.item(0)
    px = p.item(1)
    py = p.item(2)
    pz = p.item(3)
    qw = q.item(0)
    qx = q.item(1)
    qy = q.item(2)
    qz = q.item(3)

    return np.array([\
            [pw*qw - px*qx - py*qy - pz*qz], \
            [pw*qx + px*qw + py*qz - pz*qy], \
            [pw*qy - px*qz + py*qw + pz*qx], \
            [pw*qz + px*qy - py*qx + pz*qw]])

def Quat2Euler(quaternion):
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    phi = np.arctan2(2.0 * (e0 * e1 + e2 * e3), e0**2.0 + e3**2.0 - e1**2.0 - e2**2.0)
    theta = np.arcsin(2.0 * (e0 * e2 - e1 * e3))
    psi = np.arctan2(2.0 * (e0 * e3 + e1 * e2), e0**2.0 + e1**2.0 - e2**2.0 - e3**2.0)

    return np.array([[phi, theta, psi]]).T

def Euler2Quat(euler):
    phi = euler.item(0)
    theta = euler.item(1)
    psi = euler.item(2)
    e0 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)
    e1 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0) - np.sin(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0)
    e2 = np.cos(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0)
    e3 = np.sin(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) - np.cos(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)

    # ensure sign convention is kept
    quat =  np.array([[e0],[e1],[e2],[e3]])
    if quat.item(0) < 0:
        quat = -1.*quat

    return quat

def Quat2Rotation(quaternion):
    q0 = quaternion.item(0)
    qbar = quaternion[1:].reshape(-1)

    R = np.eye(3) + 2*q0*hat(qbar) + 2*(hat(qbar)@hat(qbar))

    return R

def Rotation2Quat(R):
    tr = np.trace(R)

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.
        q0 = 0.25 * S
        qx = (R[1, 2] - R[2, 1]) / S
        qy = (R[2, 0] - R[0, 2]) / S
        qz = (R[0, 1] - R[1, 0]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.
        q0 = (R[1, 2] - R[2, 1]) / S
        qx = 0.25 * S
        qy = (R[1, 0] + R[0, 1]) / S
        qz = (R[2, 0] + R[0, 2]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.
        q0 = (R[2, 0] - R[0, 2]) / S
        qx = (R[1, 0] + R[0, 1]) / S
        qy = 0.25 * S
        qz = (R[2, 1] + R[1, 2]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.
        q0 = (R[0, 1] - R[1, 0]) / S
        qx = (R[2, 0] + R[0, 2]) / S
        qy = (R[2, 1] + R[1, 2]) / S
        qz = 0.25 * S

    return np.array([[q0, qx, qy, qz]]).T


def hat(omega):
    a = omega.item(0)
    b = omega.item(1)
    c = omega.item(2)

    omega_hat = np.array([[0, -c, b],
                          [c, 0, -a],
                          [-b, a, 0]])
    return omega_hat

def vee(mat3):
    return np.array([[mat3[2,1], mat3[0,2], mat3[1,0]]]).T

def proj(v):
    return np.eye(3) - v @ v.T

def cross(a, b):
    return hat(a) @ b

def skew_sym(A):
    return 0.5 * (A - A.T)

def log(R):
    om_skew = (np.arccos((np.trace(R)-1)/2.0)/np.sqrt((3-np.trace(R))*(1+np.trace(R))))*(R-R.T)
    if np.any(np.isnan(om_skew)):
        om_skew = np.zeros((3, 3))
    return om_skew

def Log(R):
    return vee(log(R))

def J_r(theta):
    th = np.linalg.norm(theta)
    th_skew = hat(theta)
    if th < 1e-5:
        return np.eye(3)
    return np.eye(3) - (1-np.cos(th))/th**2 * th_skew + (th-np.sin(th))/th**3 * th_skew@th_skew

def J_l(theta):
    th = np.linalg.norm(theta)
    th_skew = hat(theta)
    if th < 1e-5:
        return np.eye(3)
    return np.eye(3) + (1-np.cos(th))/th**2 * th_skew + (th-np.sin(th))/th**3 * th_skew@th_skew

def J_r_inv(theta):
    th = np.linalg.norm(theta)
    th_skew = hat(theta)
    if th < 1e-5:
        return np.eye(3)
    return np.eye(3) + 1.0/2.0 * th_skew + (1.0/th**2 - (1+np.cos(th))/(2*th*np.sin(th))) * th_skew@th_skew

def J_l_inv(theta):
    th = np.linalg.norm(theta)
    th_skew = hat(theta)
    if th < 1e-5:
        return np.eye(3)
    return np.eye(3) - 1.0/2.0 * th_skew + (1.0/th**2 - (1+np.cos(th))/(2*th*np.sin(th))) * th_skew@th_skew