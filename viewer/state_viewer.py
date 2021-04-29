"""
data_viewer

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
sys.path.append('viewer/state_plotter')

from viewer.state_plotter.Plotter import Plotter
from viewer.state_plotter.plotter_args import *

from utils import Quat2Euler


class StateViewer:
    def __init__(self):
        time_window_length=200
        self.plotter = Plotter(plotting_frequency=10, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "States") # name the window
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_c'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_c'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['pz', 'pz_c'],
                              labels={'left': 'h(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots]

        # define second row
        u_plots = PlotboxArgs(plots=['u', 'u_c'],
                               labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        v_plots = PlotboxArgs(plots=['v', 'v_c'],
                               labels={'left': 'v(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        w_plots = PlotboxArgs(plots=['w', 'w_c'],
                               labels={'left': 'u(m/s)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)

        second_row = [u_plots, v_plots, w_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_c'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_c'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_c'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_c'],
                              labels={'left': 'r(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state',      ['pn', 'pe', 'pz', 'u', 'v', 'w', 'phi', 'theta', 'psi', 'p', 'q', 'r'])
        # self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'pz_e', 'u_e', 'v_e', 'w_e', 'phi_e', 'theta_e', 'psi_e', 'p_e', 'q_e', 'r_e'])
        self.plotter.define_input_vector('desired_state',   ['pn_c', 'pe_c', 'pz_c', 'u_c', 'v_c', 'w_c', 'phi_c', 'theta_c', 'psi_c', 'p_c', 'q_c', 'r_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, commanded_state, ts):

        #convert quaternions to euler angles

        euler = Quat2Euler(true_state[6:10])
        true_st_euler = np.concatenate((true_state[0:6], euler, true_state[10:13]))
        true_st_euler = np.squeeze(true_st_euler)

        # euler = Quat2Euler(estimated_state[6:10])
        # estimated_st_euler = np.concatenate((estimated_state[0:6], euler, estimated_state[10:13]))
        # estimated_st_euler = np.squeeze(estimated_st_euler)

        euler = Quat2Euler(commanded_state[6:10])
        commanded_st_euler = np.concatenate((commanded_state[0:6], euler, commanded_state[10:13]))
        commanded_st_euler = np.squeeze(commanded_st_euler)

        self.plotter.add_vector_measurement('true_state', true_st_euler, self.time)
        # self.plotter.add_vector_measurement('estimated_state', estimated_st_euler, self.time)
        self.plotter.add_vector_measurement('desired_state', commanded_st_euler, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()