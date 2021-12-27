import numpy as np
from so3_py import *

class DesiredState:
    def __init__(self, pd, pd_dot, pd_ddot, pd_dddot, psid, psid_dot):
        self.pd = pd
        self.pd_dot = pd_dot
        self.pd_ddot = pd_ddot
        self.pd_dddot = pd_dddot
        self.psid = psid
        self.psid_dot = psid_dot

class QuadGeometricController:
    def __init__(self, P):
        self.Kp = P.Kp
        self.Kv = P.Kv
        self.kr = P.kr
        self.Kom = P.Kom

        self.g = P.g
        self.m = P.m
        self.J = P.J
        self.ts = P.ts
        self.sig = P.sig

        self.om_d_dot = np.zeros((3,1))
        self.om_d_prev = np.zeros((3,1))

        self.mix_inv = np.linalg.inv(P.mixer)

    def update(self, state, des):
        p = state[0:3]
        v = state[3:6]
        R_bi = Quat2Rotation(state[6:10])
        om_b = state[10:]

        e3 = np.array([[0., 0., 1.]]).T

        p_err = p - des.pd
        pdot_err = v - des.pd_dot

        sd = np.array([[np.cos(des.psid)], [np.sin(des.psid)], [0]])
        sd_dot = des.psid_dot * np.array([[-np.sin(des.psid)], [np.cos(des.psid)], [0]])

        f = self.m * (-self.g*e3 - self.Kp@p_err - self.Kv@pdot_err + des.pd_ddot)

        # T = -np.dot(f.T, R_bi.T@e3).item(0)
        T = np.linalg.norm(f)

        k_d = -f/np.linalg.norm(f)

        kxs = cross(k_d, sd)
        j_d = kxs/np.linalg.norm(kxs)
        i_d = cross(j_d, k_d)

        R_di = np.hstack([i_d, j_d, k_d])

        pddot_err = (self.g*e3 - T/self.m * R_bi@e3 - des.pd_ddot)
        k_d_dot = proj(k_d)/np.linalg.norm(f) @ (self.m * (self.Kp@pdot_err + self.Kv@pddot_err - des.pd_dddot))
        j_d_dot = proj(j_d)/np.linalg.norm(kxs) @ (cross(k_d, sd_dot) + cross(k_d_dot, sd))
        i_d_dot = cross(j_d, k_d_dot) + cross(j_d_dot, k_d)
        R_di_dot = np.hstack([i_d_dot, j_d_dot, k_d_dot])

        om_d = vee(R_di.T @ R_di_dot)

        #have to numerically differentiate because analytical derivative is nasty
        self.om_d_dot = ((2*self.sig - self.ts)/(2*self.sig + self.ts))*self.om_d_dot + (2/(2*self.sig + self.ts))*(om_d - self.om_d_prev)
        self.om_d_prev = om_d

        R_db = R_bi.T @ R_di
        om_err = om_b - R_db@om_d

        om_db_dot = -cross(om_b, R_db@om_d) + R_db@self.om_d_dot
        r_err = Log(R_db)

        # tau = cross(om_b, self.J@om_b) + self.J@om_db_dot + self.kr*vee(skew_sym(R_db)) - self.Kom@om_err #this uses trace error
        tau = cross(om_b, self.J@om_b) + self.J@om_db_dot + R_db@J_l_inv(r_err).T@self.kr@r_err - self.Kom@om_err #this uses log error

        delta_unsat = self.mix_inv @ np.vstack([T, tau])

        commanded_state = np.vstack([des.pd, des.pd_dot, Rotation2Quat(R_di), om_d])
        
        return self.sat(delta_unsat), commanded_state

    def sat(self, outputs):
        for i in range(outputs.shape[0]):
            if outputs[i] > 1.0:
                outputs[i] = 1.0
            elif outputs[i] < 0.0:
                outputs[i] = 0.0
        return outputs
