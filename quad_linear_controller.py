import numpy as np
import control
from so3_py import *
from so3 import SO3

#this controller uses LQR for trajectory following
#and geometric control for attitude tracking
class QuadLinearController:
    def __init__(self, P):
        self.At = P.At
        self.Bt = P.Bt
        self.Kt, _, _ = control.lqr(P.At, P.Bt, P.Qt, P.Rt)
        self.Qr = P.Qr
        self.Rr = P.Rr

        self.p_err_int = np.zeros((3,1))
        self.p_err_prev = np.zeros((3,1))

        self.Kom = P.Kom
        self.kr = P.kr
        self.kr_lee1 = P.kr_lee1
        self.kr_lee2 = P.kr_lee2
        
        self.kr1_lee3 = P.kr1_lee3
        self.kr2_lee3 = P.kr2_lee3
        self.alpha_lee3 = P.alpha_lee3
        self.beta_lee3 = P.beta_lee3
        self.delta_lee3 = P.delta_lee3
        self.m_lee3 = 0 #can be 0, 1, or 2. 0 is nominal
        self.om_lim_lee3 = P.om_lim_lee3
        self.Kom_lee3 = P.Kom_lee3

        self.control_type = P.control_type

        self.g = P.g
        self.m = P.m
        self.J = P.J
        self.ts = P.ts
        self.sig = P.sig

        self.om_d_dot = np.zeros((3,1))
        self.om_d_prev = np.zeros((3,1))

        self.mix_inv = np.linalg.inv(P.control_mixer)

        self.clipped_props = P.clipped_props
        self.clipped_mix_inv = np.linalg.inv(P.mixer[[0,2],:2])

    def update(self, state, des):
        p = state[0:3]
        v = state[3:6]
        R_bi = Quat2Rotation(state[6:10])
        om_b = state[10:]

        e3 = np.array([[0., 0., 1.]]).T

        p_err = p - des.pd
        pdot_err = v - des.pd_dot

        self.p_err_int += self.ts/2.0 * (p_err + self.p_err_prev)
        self.p_err_prev = p_err

        et = np.vstack([p_err, pdot_err, self.p_err_int])

        # et = np.vstack([p_err, pdot_err])

        if not self.clipped_props:
            psid = des.psid
            psid_dot = des.psid_dot
        else:
            euler = Quat2Euler(state[6:10])
            psid = euler.item(2)
            psid_dot = om_b[-1]

        sd = np.array([[np.cos(psid)], [np.sin(psid)], [0]])
        sd_dot = psid_dot * np.array([[-np.sin(psid)], [np.cos(psid)], [0]])

        f = -self.Kt @ et + self.m*(self.g*e3 - des.pd_ddot)

        # T = -np.dot(f.T, R_bi.T@e3).item(0)
        T = np.linalg.norm(f)

        k_d = f/np.linalg.norm(f)
        kxs = cross(k_d, sd)
        j_d = kxs/np.linalg.norm(kxs)
        i_d = cross(j_d, k_d)

        R_di = np.hstack([i_d, j_d, k_d])

        k_d_dot = proj(k_d)/np.linalg.norm(f) @ (-self.m*des.pd_dddot - self.Kt@(self.At - self.Bt@self.Kt)@et)
        j_d_dot = proj(j_d)/np.linalg.norm(kxs) @ (cross(k_d, sd_dot) + cross(k_d_dot, sd))
        i_d_dot = cross(j_d, k_d_dot) + cross(j_d_dot, k_d)

        R_di_dot = np.hstack([i_d_dot, j_d_dot, k_d_dot])

        om_d = vee(R_di.T @ R_di_dot)

        #have to numerically differentiate because analytical derivative is nasty
        self.om_d_dot = ((2*self.sig - self.ts)/(2*self.sig + self.ts))*self.om_d_dot + (2/(2*self.sig + self.ts))*(om_d - self.om_d_prev)
        self.om_d_prev = om_d

        R_db = R_bi.T @ R_di
        om_err = R_db@om_d - om_b
        r_err = np.array([SO3(R_db).Log().tolist()]).T

        om_db_dot = -cross(om_b, R_db@om_d) + R_db@self.om_d_dot

        if self.control_type == "lee1":
          tau = cross(om_b, self.J@om_b) + self.J@om_db_dot + self.kr_lee1*vee(skew_sym(R_db)) + self.Kom@om_err #this uses trace error

        elif self.control_type == "lee2":
          tau = cross(R_db@om_d, self.J@R_db@om_d) + self.J@R_db@self.om_d_dot + self.kr_lee2*vee(skew_sym(R_db))/np.sqrt(1.0+np.trace(R_db)) + self.Kom@om_err #this uses wierd trace error

        elif self.control_type == "lee3":
          r_1d = R_di[:,[0]]
          r_2d = R_di[:,[1]]
          r_3d = R_di[:,[2]]
          r_1b = R_bi[:,[0]]
          r_2b = R_bi[:,[1]]
          e_r1 = cross(R_bi.T @ r_1d, np.array([[1, 0, 0]]).T)
          e_r2 = cross(R_bi.T @ r_2d, np.array([[0, 1, 0]]).T)

          Psi_n1 = 1 - (r_1b.T @ r_1d).item(0)
          Psi_n2 = 1 - (r_2b.T @ r_2d).item(0)
          Psi_e1 = self.alpha_lee3 + self.beta_lee3 * (r_1b.T @ r_3d).item(0)
          Psi_e2 = self.alpha_lee3 + self.beta_lee3 * (r_2b.T @ r_3d).item(0)
          Psi_1 = self.kr1_lee3 * Psi_n1 + self.kr2_lee3 * Psi_n2
          Psi_2 = self.kr1_lee3 * Psi_n1 + self.kr2_lee3 * Psi_e2
          Psi_3 = self.kr1_lee3 * Psi_e1 + self.kr2_lee3 * Psi_n2
          Psi_choices = [Psi_1, Psi_2, Psi_3]
          Psi_min = np.argmin(Psi_choices)
          rho = Psi_choices[Psi_min]
          if Psi_choices[self.m_lee3] - rho >= self.delta_lee3 and np.linalg.norm(om_err) < self.om_lim_lee3:
            self.m_lee3 = Psi_min

          e_h1 = e_r1 if self.m_lee3 !=2 else -self.beta_lee3 * cross(R_bi.T @ r_3d, np.array([[1, 0, 0]]).T)
          e_h2 = e_r2 if self.m_lee3 !=1 else -self.beta_lee3 * cross(R_bi.T @ r_3d, np.array([[0, 1, 0]]).T)
          e_h = self.kr1_lee3*e_h1 + self.kr2_lee3*e_h2
          tau = cross(R_db@om_d, self.J@R_db@om_d) + self.J@R_db@self.om_d_dot - e_h + self.Kom_lee3@om_err # this uses the overly complicated global hybrid controller

        elif self.control_type == "ours":
          tau = cross(om_b, self.J@om_b) + self.J@om_db_dot + SO3.Jl_inv(r_err).T@self.kr@r_err + self.Kom@om_err #this uses log error

        commanded_state = np.vstack([des.pd, des.pd_dot, Rotation2Quat(R_di), om_d])
        delta_unsat = self.mix_inv @ np.vstack([T, tau])

        return self.sat(delta_unsat), commanded_state

    def sat(self, outputs):
        for i in range(outputs.shape[0]):
            if outputs[i] > 1.0:
                outputs[i] = 1.0
            elif outputs[i] < 0.0:
                outputs[i] = 0.0
        return outputs

