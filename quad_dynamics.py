import numpy as np
from utils import *

class QuadDynamics:
  def __init__(self, P):
    self.state = P.x0 #[p,v,q,om].T
    self.m = P.m
    self.J = P.J
    self.J_inv = np.linalg.inv(self.J)
    self.g = P.g
    self.ts = P.ts
    self.D = P.D #linear drag matrix
    self.A = P.A #angular drag matrix
    self.kf = P.kf #force coefficient
    self.kt = P.kt #torque coefficient
    self.l = P.l #arm length

    self.input_stdev = P.input_stdev

    self.mixer = P.mixer

    self.clipper_props = P.clipped_props

  def update(self, u):
    u += np.random.normal(np.zeros((4,1)), self.input_stdev*np.ones((4,1)))
    u = self.sat(u)

    fm = self.force_moment(u)
    k1 = self.derivatives(self.state, fm)
    k2 = self.derivatives(self.state + self.ts/2.*k1, fm)
    k3 = self.derivatives(self.state + self.ts/2.*k2, fm)
    k4 = self.derivatives(self.state + self.ts*k3, fm)
    self.state += self.ts/6.0 * (k1 + 2*k2 + 2*k3 + k4)

    self.state[6:10] /= np.linalg.norm(self.state[6:10])


  def derivatives(self, state, fm):
    p = state[0:3]
    v = state[3:6]
    q = state[6:10]
    om = state[10:]

    f = fm[0:3]
    m = fm[3:]

    pdot = v

    e3 = np.array([[0],[0],[1]])
    vdot = self.g*e3 + Quat2Rotation(q) @ f/self.m #forces are in the body frame

    # qdot = 0.5*QuatTimes(np.vstack([[0], -om]), q)
    qdot = 0.5*QuatTimes(q, np.vstack([[0], om]))

    omdot = self.J_inv @ (-cross(om, self.J@om) + m)

    return np.vstack([pdot, vdot, qdot, omdot])

  def force_moment(self, delta): #returns body frame wrench
    R_bi = Quat2Rotation(self.state[6:10])
    om = self.state[10:]

    #ignore inputs 3 and 4 if the props are clipped
    if self.clipper_props:
        fm_motor = self.mixer[:,:2] @ delta[:2]
    else:
        fm_motor = self.mixer @ delta
    f = np.array([[0],[0],[-1]]) * fm_motor.item(0)

    f -= self.D @ R_bi.T @ self.state[3:6]

    m = fm_motor[1:]
    # m += -np.sign(om) * self.A @ (om * om) #hackish, but gets the job done

    return np.vstack([f, m])

  def sat(self, inputs):
      for i in range(inputs.shape[0]):
          if inputs[i] > 1.0:
              inputs[i] = 1.0
          elif inputs[i] < 0.0:
              inputs[i] = 0.0
      return inputs