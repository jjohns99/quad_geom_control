import numpy as np
import time
import params as P
from quad_dynamics import QuadDynamics
from quad_geometric_controller import *
from quad_linear_controller import QuadLinearController
from quad_se3_controller import QuadSE3Controller
from trajectory_generator import TrajectoryGenerator
from viewer.quad_viewer import QuadViewer
from viewer.state_viewer import StateViewer
from viewer.controls_viewer import ControlsViewer
from so3 import Euler2Quat

def main():
    dyn = QuadDynamics(P)
    # ctrl = QuadGeometricController(P)
    ctrl = QuadLinearController(P)
    # ctrl = QuadSE3Controller(P)

    traj = TrajectoryGenerator()
    traj.create_sin_traj(P.period, P.amplitude, P.phase, P.offset)
    # traj.create_bspline_traj(P.degree, P.knots, P.coeffs)
    traj_points = traj.get_plot_points()

    view = QuadViewer(P.l)
    view.addTrajectory(traj_points)
    # state_plots = StateViewer()
    # ctrl_plots = ControlsViewer()

    save_data = None

    # delta = np.array([[0.5],[0.5],[0.5],[0.5]])
    t = 0
    last_time = time.time()
    input()
    while t < P.tf:
        des_state = traj.get_state(t)
        # des_state = DesiredState(np.array([[0.0, 0.0, 0.0]]).T, np.array([[0.0, 0.0, 0.0]]).T, np.zeros((3,1)), np.zeros((3,1)), 0.0, 0.0)
        delta, commanded_state = ctrl.update(dyn.state, des_state)
        dyn.update(delta)
        commanded_state[7:10] = -commanded_state[7:10]

        view.update(dyn.state)
        # state_plots.update(dyn.state, commanded_state, P.ts)
        # ctrl_plots.update(delta, P.ts)
        t += P.ts
        while time.time() - last_time < P.ts:
            pass
        last_time = time.time()

        if save_data is None:
          save_data = np.hstack([np.array([[t]]), dyn.state.T, commanded_state.T])
          print(save_data.shape)
        else: 
          save_data = np.vstack([save_data, np.hstack([np.array([[t]]), dyn.state.T, commanded_state.T])])

    # np.save("data/upside_down_trace.npy", save_data)

if __name__ == "__main__":
    main()