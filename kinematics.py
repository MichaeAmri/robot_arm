import numpy as np
from math import sin, cos, pi

class robot_arm:

    def __init__(self):

        self.h1 = 50
        self.l34 = 300
        self.l56 = 200
        self.l1 = 40
        self.h2 = 150

        self.t_twist = np.array([0,         pi/2,       0,      0,          0,      pi/2])
        self.s_shift = np.array([self.h1,   0,          0,      self.l34,   0,      self.l56])
        self.a_shift = np.array([self.l1,   self.h2,    0,      0,          0,      0])
        self.a_twist = np.array([pi/2,      0,          pi/2,   -pi/2,      pi/2,   0])

        self.generalized_coordinate_vector = np.array([0, 0, 0, 0, 0, 0])


    def transition_matrix(self, start_system=0, target_system=6):
        
        """
        transition matrix from coordinate starting coordinate system to target system
        """

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
        return self.transition_matrix().dot(np.array([0, 0, 0, 1]))[:3].round(2)

if __name__ == '__main__':

    arm = robot_arm()
    test_vector = np.array([0, 0, 0, 1])

    print (arm.get_end_effector_coordinates())

