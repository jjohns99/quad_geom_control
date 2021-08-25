import numpy as np
from scipy.interpolate import BSpline
from so3 import *
from quad_geometric_controller import DesiredState
import matplotlib.pyplot as plt

class TrajectoryGenerator:
    def __init__(self):
        pass

    def create_sin_traj(self, period, amplitude, phase, offset):
        self.type = "sinusoid"
        self.period = period
        self.amplitude = amplitude
        self.phase = phase
        self.offset = offset

    def create_bspline_traj(self, degree, knots, coeffs):
        self.type = "bspline"
        self.knots = knots
        self.x_spline = BSpline(knots, coeffs[0,:], degree)
        self.z_spline = BSpline(knots, coeffs[1,:], degree)
        self.x_derivs = []
        self.z_derivs = []
        for i in range(3):
            self.x_derivs.append(self.x_spline.derivative(i+1))
            self.z_derivs.append(self.z_spline.derivative(i+1))

    def get_state(self, t):
        if self.type == "sinusoid":
            p = self.amplitude * np.sin(2.0*np.pi/self.period*t + self.phase) + self.offset
            p_dot = self.amplitude * (2.0*np.pi/self.period) * np.cos(2.0*np.pi/self.period*t + self.phase)
            p_ddot = -self.amplitude * (2.0*np.pi/self.period)**2 * np.sin(2.0*np.pi/self.period*t + self.phase)
            p_dddot = -self.amplitude * (2.0*np.pi/self.period)**3 * np.cos(2.0*np.pi/self.period*t + self.phase)

            vx = p_dot.item(0)
            vy = p_dot.item(1)
            ax = p_ddot.item(0)
            ay = p_ddot.item(1)
            psi = 0.0 #np.arctan2(vy, vx)
            psi_dot = 0.0 #(vx*ay + vy*ax)/(vx**2 + vy**2)
            # psi = 10.*t
            # psi_dot = 10.0

            return DesiredState(p, p_dot, p_ddot, p_dddot, psi, psi_dot)

        elif self.type == "bspline":
            if t > self.knots[-1]:
                # p = np.array([[self.x_spline(self.knots[-1]).item(0), 0.0, self.z_spline(self.knots[-1]).item(0)]]).T
                p = np.array([[0.0, self.x_spline(self.knots[-1]).item(0), self.z_spline(self.knots[-1]).item(0)]]).T
                derivs = []
                for i in range(3):
                    derivs.append(np.zeros((3,1)))
            else:
                # p = np.array([[self.x_spline(t).item(0), 0.0, self.z_spline(t).item(0)]]).T
                p = np.array([[0.0, self.x_spline(t).item(0), self.z_spline(t).item(0)]]).T
                derivs = []
                for i in range(3):
                    # derivs.append(np.array([[self.x_derivs[i](t).item(0), 0.0, self.z_derivs[i](t).item(0)]]).T)
                    derivs.append(np.array([[0.0, self.x_derivs[i](t).item(0), self.z_derivs[i](t).item(0)]]).T)
            vx = derivs[0].item(0)
            vy = derivs[0].item(1)
            ax = derivs[1].item(0)
            ay = derivs[1].item(1)
            # psi = np.arctan2(vy, vx)
            # if (vx**2 + vy**2) == 0:
            #     psi_dot = 0.0
            # else:
            #     psi_dot = (vx*ay + vy*ax)/(vx**2 + vy**2)
            psi = 0.0
            psi_dot = 0.0

            return DesiredState(p, derivs[0], derivs[1], derivs[2], psi, psi_dot)

    def get_plot_points(self):
        if self.type == "sinusoid":
            t = np.linspace(0.0, np.max(self.period), 1000)
            return self.amplitude * np.sin(2.0*np.pi/self.period*t + self.phase) + self.offset

        elif self.type == "bspline":
            t = np.linspace(self.knots[0], self.knots[-1], 1000)
            # plt.subplot(421)
            # plt.plot(t, self.x_spline(t))
            # plt.subplot(422)
            # plt.plot(t, self.z_spline(t))
            # plt.subplot(423)
            # plt.plot(t, self.x_derivs[0](t))
            # plt.subplot(424)
            # plt.plot(t, self.z_derivs[0](t))
            # plt.subplot(425)
            # plt.plot(t, self.x_derivs[1](t))
            # plt.subplot(426)
            # plt.plot(t, self.z_derivs[1](t))
            # plt.subplot(427)
            # plt.plot(t, self.x_derivs[2](t))
            # plt.subplot(428)
            # plt.plot(t, self.z_derivs[2](t))
            # plt.show()
            # input()
            return np.vstack([np.zeros(t.shape), self.x_spline(t), self.z_spline(t)])
            # return np.vstack([self.x_spline(t), np.zeros(t.shape), self.z_spline(t)])