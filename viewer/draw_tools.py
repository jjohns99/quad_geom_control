import numpy as np
import pyqtgraph.opengl as gl
from utils import Quat2Rotation

class drawQuad():
    def __init__(self, state, window, arm_length):
        self.unit_length = arm_length/10.0

        self.bodybox_points, self.bodybox_meshColors = self.get_bodybox_points()

        self.motor_relative_positions = [np.array([[10*self.unit_length, 0., 0.]]).T, 
                                        np.array([[-10*self.unit_length, 0., 0.]]).T,
                                        np.array([[0., 10*self.unit_length, 0.]]).T,
                                        np.array([[0., -10*self.unit_length, 0.]]).T]
        self.motor_points = []
        self.motor_meshColors = []
        for i in range(4):
            if i == 0:
                color = np.array([1., 0., 0., 1.])
            else:
                color = np.array([0., 1., 0., 1.])
            points, meshColors = self.get_motor_points(color)
            self.motor_points.append(self.translate_points(points, self.motor_relative_positions[i]))
            self.motor_meshColors.append(meshColors)

        quad_position = state[0:3] # NED coordinates
        # attitude of quad as a rotation matrix R from body to inertial
        R_bi = Quat2Rotation(state[6:10])
        # convert North-East Down to East-North-Up for rendering
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # bodybox
        rotated_body = self.rotate_points(self.bodybox_points, R_bi)
        translated_body = self.translate_points(rotated_body, quad_position)
        translated_body = R_ned @ translated_body
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        body_mesh = self.bodybox_points_to_mesh(translated_body)
        self.quad_body = gl.GLMeshItem(vertexes=body_mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.bodybox_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.quad_body)  # add body to plot

        #motors
        self.quad_motors = []
        for i in range(4):
            rotated_motor = self.rotate_points(self.motor_points[i], R_bi)
            translated_motor = self.translate_points(rotated_motor, quad_position)
            translated_motor = R_ned @ translated_motor
            motor_mesh = self.motor_points_to_mesh(translated_motor)
            self.quad_motors.append(gl.GLMeshItem(vertexes=motor_mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.motor_meshColors[i],  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False))  # speeds up rendering
            window.addItem(self.quad_motors[i])

    def update(self, state):
        quad_position = state[0:3]
        R_bi = Quat2Rotation(state[6:10])
        R_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        # bodybox
        rotated_body = self.rotate_points(self.bodybox_points, R_bi)
        translated_body = self.translate_points(rotated_body, quad_position)
        translated_body = R_ned @ translated_body
        body_mesh = self.bodybox_points_to_mesh(translated_body)
        self.quad_body.setMeshData(vertexes=body_mesh, vertexColors=self.bodybox_meshColors)

        for i in range(4):
            rotated_motor = self.rotate_points(self.motor_points[i], R_bi)
            translated_motor = self.translate_points(rotated_motor, quad_position)
            translated_motor = R_ned @ translated_motor
            motor_mesh = self.motor_points_to_mesh(translated_motor)
            self.quad_motors[i].setMeshData(vertexes=motor_mesh, vertexColors=self.motor_meshColors[i])

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_bodybox_points(self):
        box_height = 4*self.unit_length
        box_width = 6*self.unit_length
        # points are in NED coordinates
        points = np.array([[box_width/2, box_width/2, box_height/2],
                          [-box_width/2, box_width/2, box_height/2],
                          [-box_width/2, -box_width/2, box_height/2],
                          [box_width/2, -box_width/2, box_height/2],
                          [box_width/2, box_width/2, -box_height/2],
                          [-box_width/2, box_width/2, -box_height/2],
                          [-box_width/2, -box_width/2, -box_height/2],
                          [box_width/2, -box_width/2, -box_height/2]
                           ]).T

        #define mesh colors for 12 triangles
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1.,1.,0.,1.])
        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = blue 
        meshColors[1] = blue
        meshColors[2] = blue
        meshColors[3] = blue
        meshColors[4] = blue
        meshColors[5] = blue
        meshColors[6] = blue
        meshColors[7] = blue
        meshColors[8] = blue
        meshColors[9] = blue
        meshColors[10] = yellow
        meshColors[11] = yellow
        return points, meshColors

    def bodybox_points_to_mesh(self, points):
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # top (in NED, so bottom in ENU)
                         [points[2], points[3], points[0]],  # top
                         [points[0], points[1], points[5]],  # right
                         [points[5], points[4], points[0]],  # right
                         [points[1], points[2], points[6]],  # front
                         [points[6], points[5], points[1]],  # front
                         [points[2], points[3], points[7]],  # left
                         [points[7], points[6], points[2]],  # left
                         [points[0], points[3], points[7]],  # back
                         [points[7], points[4], points[0]],  # back
                         [points[4], points[5], points[6]],  # bottom
                         [points[6], points[7], points[4]],  # bottom
                         ])
        return mesh

    def get_motor_points(self, color):
        rad = 4*self.unit_length

        c = np.cos(30*np.pi/180)
        s = np.sin(30*np.pi/180)
        # points are in NED coordinates
        points = np.array([[0, 0, 0],  # [0]
                           [rad*c, rad*s, 0],  # [1]
                           [0, rad, 0],  # [2]
                           [-rad*c, rad*s, 0],  # [3]
                           [-rad*c, -rad*s, 0],  # [4]
                           [0, -rad, 0],  # [5]
                           [rad*c, -rad*s, 0]  # [6]
                           ]).T

        meshColors = np.empty((6, 3, 4), dtype=np.float32)
        meshColors[0] = color
        meshColors[1] = color
        meshColors[2] = color 
        meshColors[3] = color
        meshColors[4] = color
        meshColors[5] = color
        return points, meshColors

    def motor_points_to_mesh(self, points):
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],
                         [points[0], points[2], points[3]],
                         [points[0], points[3], points[4]],
                         [points[0], points[4], points[5]],
                         [points[0], points[5], points[6]],
                         [points[0], points[6], points[1]]
                         ])
        return mesh


class drawTrajectory:
    def __init__(self, points, color, window):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        
        points = points.T
        self.color = color
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object =  gl.GLLinePlotItem(pos=points,
                                                   color=path_color,
                                                   width=2,
                                                   antialias=True,
                                                   mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, points):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        self.path_plot_object.setData(pos=points)
