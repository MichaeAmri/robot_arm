import numpy as np
from math import sin, cos, pi

class robot_arm:

    def __init__(self):

        '''
        see the dimensions of the links and the location
        of the coordinate systems in the file Schematics.pdf
        '''

        # dimentions
        self.h1 = 50
        self.l34 = 300
        self.l56 = 200
        self.l1 = 40
        self.h2 = 150

        # Denavit-Hartenberg parameters
        self.t_twist = np.array([0,         pi/2,       0,      0,          0,      pi/2])
        self.s_shift = np.array([self.h1,   0,          0,      self.l34,   0,      self.l56])
        self.a_shift = np.array([self.l1,   self.h2,    0,      0,          0,      0])
        self.a_twist = np.array([pi/2,      0,          pi/2,   -pi/2,      pi/2,   0])

        self.generalized_coordinate_vector = np.array([0, 0, 0, 0, 0, 0])

    def transition_matrix(self, start_system=0, target_system=6):
        
        '''
        transition matrix from coordinate starting coordinate system to target system
        '''

        self.t_twist = self.t_twist + self.generalized_coordinate_vector

        T = np.eye(4)

        for i in range(start_system, target_system):

            A = np.array([
                [cos(self.t_twist[i]),  -sin(self.t_twist[i]) * cos(self.a_twist[i]),   sin(self.t_twist[i]) * sin(self.a_twist[i]),    self.a_shift[i] * cos(self.t_twist[i])],
                [sin(self.t_twist[i]),  cos(self.t_twist[i]) * cos(self.a_twist[i]),    -cos(self.t_twist[i]) * sin(self.a_twist[i]),   self.a_shift[i] * sin(self.t_twist[i])],
                [0,                     sin(self.a_twist[i]),                           cos(self.a_twist[i]),                           self.s_shift[i]],
                [0,                     0,                                              0,                                              1]
            ])

            T = T.dot(A)

        return T

    def get_end_effector_coordinates(self):
        return self.transition_matrix().dot(np.array([0, 0, 0, 1]))[:3]

    def get_end_effector_angles(self):
        return self.transition_matrix().diagonal()[:3]

    def get_n_link_coordinates(self, n):
        return self.transition_matrix(target_system=n).dot(np.array([0, 0, 0, 1]))[:3]


if __name__ == '__main__':

    import visualization as vis

    arm = robot_arm()
    print (arm.get_end_effector_coordinates().round(2))
    print (arm.get_end_effector_angles().round(2))
    vis.draw_robot(arm)

