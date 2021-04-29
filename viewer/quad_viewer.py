import sys
sys.path.append("..")
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
import numpy as np
from viewer.draw_tools import drawQuad, drawTrajectory


class QuadViewer:
    def __init__(self, arm_length):
        self.arm_length = arm_length
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('Quad Viewer')
        self.window.setGeometry(0, 0, 1280, 720)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(2, 2, 2) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=50) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the mav been plotted yet?
        self.quad_plot = []

    def update(self, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.quad_plot = drawQuad(state, self.window, self.arm_length)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.quad_plot.update(state)
        # update the center of the camera view to the mav location
        view_location = Vector(state.item(1), state.item(0), -state.item(2))  # defined in ENU coordinates
        self.window.opts['center'] = view_location

        self.tick()

    def tick(self):
        # redraw
        self.app.processEvents()

    def addTrajectory(self, points):
        blue = np.array([[30, 144, 255, 255]])/255.
        self.trajectory = drawTrajectory(points, blue, self.window)
