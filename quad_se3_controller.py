import numpy as np
import control
import so3
import se3

class QuadSE3Controller:
    def __init__(self, P):
        self.g = P.g
        self.m = P.m
        self.J = P.J
        self.ts = P.ts
        self.sig = P.sig

        self.Kt = P.Kt
        self.Kv = P.Kv
        self.Kom = P.Kom

        # initialize desired rotation to body rotation in first control loop pass
        self.init_des_rot = False
        self.R_di = np.eye(3)
        self.om_d = np.zeros((3,1))

        self.om_d_dot = np.zeros((3,1))
        self.om_d_prev = np.zeros((3,1))

        self.nasty_term_dot = np.zeros((3,1))
        self.nasty_term_prev = np.zeros((3,1))

        self.mix_inv = np.linalg.inv(P.mixer)

    def update(self, state, des):
        p = state[0:3]
        v = state[3:6]
        R_bi = so3.Quat2Rotation(state[6:10])
        om_b = state[10:]

        e3 = np.array([[0.0, 0.0, 1.0]]).T

        if not self.init_des_rot:
            self.R_di = R_bi
            self.om_d = om_b
            self.init_des_rot = True
        
        T_bi = np.block([[R_bi, p], [np.zeros((1,3)), 1.0]])
        T_di = np.block([[self.R_di, des.pd], [np.zeros((1,3)), 1.0]])

        T_db = se3.inverse(T_bi) @ T_di
        T_err = se3.Log(T_db)
        pdot_err = v - des.pd_dot

        R_db = T_db[:3,:3]
        om_err = om_b - R_db@self.om_d

        #have to numerically differentiate because analytical derivative is nasty
        self.om_d_dot = ((2*self.sig - self.ts)/(2*self.sig + self.ts))*self.om_d_dot + (2/(2*self.sig + self.ts))*(self.om_d - self.om_d_prev)
        self.om_d_prev = self.om_d

        om_db_dot = -so3.cross(om_b, R_db@self.om_d) + R_db@self.om_d_dot

        wrench = np.vstack([-self.g * e3 + des.pd_ddot, so3.cross(om_b, self.J@om_b) + self.J@om_db_dot]) \
                    + np.block([[self.R_di, np.zeros((3,3))], [np.zeros((3,3)), R_db]])@se3.J_l_inv(T_err).T@self.Kt@T_err \
                    - np.vstack([self.Kv@pdot_err, self.Kom@om_err])

        sd = np.array([[np.cos(des.psid)], [np.sin(des.psid)], [0]])
        sd_dot = des.psid_dot * np.array([[-np.sin(des.psid)], [np.cos(des.psid)], [0]])

        f = wrench[:3] * self.m
        k_d = -f/np.linalg.norm(f)
        kxs = so3.cross(k_d, sd)
        j_d = kxs/np.linalg.norm(kxs)
        i_d = so3.cross(j_d, k_d)

        self.R_di = np.hstack([i_d, j_d, k_d])

        pddot_err = self.g*e3 + f/self.m - des.pd_ddot
        nasty_term = np.block([self.R_di, np.zeros((3,3))]) @ se3.J_l_inv(T_err).T @ self.Kt @ T_err
        self.nasty_term_dot = ((2*self.sig - self.ts)/(2*self.sig + self.ts))*self.nasty_term_dot + (2/(2*self.sig + self.ts))*(nasty_term - self.nasty_term_prev)
        self.nasty_term_prev = nasty_term

        k_d_dot = -so3.proj(k_d)/np.linalg.norm(f) * self.m * (des.pd_dddot - self.Kv@pddot_err + self.nasty_term_dot)
        j_d_dot = so3.proj(j_d)/np.linalg.norm(kxs) @ (so3.cross(k_d, sd_dot) + so3.cross(k_d_dot, sd))
        i_d_dot = so3.cross(j_d, k_d_dot) + so3.cross(j_d_dot, k_d)

        R_di_dot = np.hstack([i_d_dot, j_d_dot, k_d_dot])

        self.om_d = so3.vee(self.R_di.T @ R_di_dot)

        thrust = np.linalg.norm(f)
        tau = wrench[3:]

        commanded_state = np.vstack([des.pd, des.pd_dot, so3.Rotation2Quat(self.R_di), self.om_d])

        delta_unsat = self.mix_inv @ np.vstack([thrust, tau])

        return self.sat(delta_unsat), commanded_state


    def sat(self, outputs):
        for i in range(outputs.shape[0]):
            if outputs[i] > 1.0:
                outputs[i] = 1.0
            elif outputs[i] < 0.0:
                outputs[i] = 0.0
        return outputs