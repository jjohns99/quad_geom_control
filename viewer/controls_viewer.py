"""
controls_viewer
plot the control inputs

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
from viewer.state_plotter.Plotter import Plotter
from viewer.state_plotter.plotter_args import *

class ControlsViewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=10, # refresh plot every 100 time steps
                               time_window=time_window_length, # plot last time_window seconds of data
                               window_title = "Controls") # name the window
        # set up the plot window
        # define first row
        m1_plot = PlotboxArgs(plots=['Front motor'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        m2_plot = PlotboxArgs(plots=['Rear motor'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        first_row = [m1_plot, m2_plot]

        # define first row
        m3_plot = PlotboxArgs(plots=['Left motor'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        m4_plot = PlotboxArgs(plots=['Right motor'],
                               labels={'left': 'delta_t', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        second_row = [m3_plot, m4_plot]


        plots = [first_row,
                 second_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('controls', ['Front motor', 'Rear motor', 'Left motor', 'Right motor'])

        # plot timer
        self.time = 0.

    def update(self, controls, ts):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        controls_list = [controls.item(0), 
                controls.item(1), 
                controls.item(2), 
                controls.item(3)]

        self.plotter.add_vector_measurement('controls', controls_list, self.time)

        self.tick()

        # increment time
        self.time += ts

    def tick(self):
        # Update and display the plot
        self.plotter.update_plots()
