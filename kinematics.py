import numpy as np
from math import sin, cos, pi

class robot_arm:

    def __init__(self):

        '''
        see the dimensions of the links and the location
        of the coordinate systems in the file Schematics.pdf
        '''

        self.generalized_coordinate_vector = np.array([0, 0, 0, 0, 0, 0])

        # dimentions
        self.h1 = 25
        self.l34 = 75
        self.l56 = 50
        self.l1 = 50
        self.h2 = 100

        # Denavit-Hartenberg parameters
        self.t_twist = np.array([0,         pi/2,       0,      0,          0,      pi/2])
        self.s_shift = np.array([self.h1,   0,          0,      self.l34,   0,      self.l56])
        self.a_shift = np.array([self.l1,   self.h2,    0,      0,          0,      0])
        self.a_twist = np.array([pi/2,      0,          pi/2,   -pi/2,      pi/2,   0])

        self.number_of_joints = len(self.t_twist)

    def derivative_matrix(self, actuated_joint_number, linear=False):

        U = np.eye(4)

        omega_linear = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, -1],
            [0, 0, 0, 0]
            ])

        omega_rotation = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
            ])

        q_twist = self.t_twist + self.generalized_coordinate_vector

        for i in range(self.number_of_joints):

            A = np.array([
                [cos(q_twist[i]),   -sin(q_twist[i]) * cos(self.a_twist[i]),    sin(q_twist[i]) * sin(self.a_twist[i]),     self.a_shift[i] * cos(q_twist[i])],
                [sin(q_twist[i]),   cos(q_twist[i]) * cos(self.a_twist[i]),     -cos(q_twist[i]) * sin(self.a_twist[i]),    self.a_shift[i] * sin(q_twist[i])],
                [0,                 sin(self.a_twist[i]),                       cos(self.a_twist[i]),                       self.s_shift[i]],
                [0,                 0,                                          0,                                          1]
            ])

            if i == actuated_joint_number:
                if linear:
                    A = omega_linear.dot(A)
                else:
                    A = omega_rotation.dot(A)

            U = U.dot(A)

        return U

    def inverse_kinematics(self, target_x=160, target_y=0, target_z=120, target_cos_yx=0, target_cos_zx=1, target_cos_zy=0):

        U = np.zeros((self.number_of_joints, 4, 4))
        count = 0
        eps = 0.0001

        while count < 100:

            for i in range(self.number_of_joints):
                U[i] = self.derivative_matrix(i)

            syst = np.array([
                [U[0, 0, 1], U[1, 0, 1], U[2, 0, 1], U[3, 0, 1], U[4, 0, 1], U[5, 0, 1]],
                [U[0, 0, 2], U[1, 0, 2], U[2, 0, 2], U[3, 0, 2], U[4, 0, 2], U[5, 0, 2]],
                [U[0, 0, 3], U[1, 0, 3], U[2, 0, 3], U[3, 0, 3], U[4, 0, 3], U[5, 0, 3]],
                [U[0, 1, 2], U[1, 1, 2], U[2, 1, 2], U[3, 1, 2], U[4, 1, 2], U[5, 1, 2]],
                [U[0, 1, 3], U[1, 1, 3], U[2, 1, 3], U[3, 1, 3], U[4, 1, 3], U[5, 1, 3]],
                [U[0, 2, 3], U[1, 2, 3], U[2, 2, 3], U[3, 2, 3], U[4, 2, 3], U[5, 2, 3]],
            ])

            solve = np.array([
                [target_cos_yx - self.transition_matrix()[0, 1] +
                U[0, 0, 1] * self.generalized_coordinate_vector[0] +
                U[1, 0, 1] * self.generalized_coordinate_vector[1] +
                U[2, 0, 1] * self.generalized_coordinate_vector[2] +
                U[3, 0, 1] * self.generalized_coordinate_vector[3] +
                U[4, 0, 1] * self.generalized_coordinate_vector[4] +
                U[5, 0, 1] * self.generalized_coordinate_vector[5]],

                [target_cos_zx - self.transition_matrix()[0, 2] +
                U[0, 0, 2] * self.generalized_coordinate_vector[0] +
                U[1, 0, 2] * self.generalized_coordinate_vector[1] +
                U[2, 0, 2] * self.generalized_coordinate_vector[2] +
                U[3, 0, 2] * self.generalized_coordinate_vector[3] +
                U[4, 0, 2] * self.generalized_coordinate_vector[4] +
                U[5, 0, 2] * self.generalized_coordinate_vector[5]],

                [target_x - self.transition_matrix()[0, 3] +
                U[0, 0, 3] * self.generalized_coordinate_vector[0] +
                U[1, 0, 3] * self.generalized_coordinate_vector[1] +
                U[2, 0, 3] * self.generalized_coordinate_vector[2] +
                U[3, 0, 3] * self.generalized_coordinate_vector[3] +
                U[4, 0, 3] * self.generalized_coordinate_vector[4] +
                U[5, 0, 3] * self.generalized_coordinate_vector[5]],

                [target_cos_zy - self.transition_matrix()[1, 2] +
                U[0, 1, 2] * self.generalized_coordinate_vector[0] +
                U[1, 1, 2] * self.generalized_coordinate_vector[1] +
                U[2, 1, 2] * self.generalized_coordinate_vector[2] +
                U[3, 1, 2] * self.generalized_coordinate_vector[3] +
                U[4, 1, 2] * self.generalized_coordinate_vector[4] +
                U[5, 1, 2] * self.generalized_coordinate_vector[5]],

                [target_y - self.transition_matrix()[1, 3] +
                U[0, 1, 3] * self.generalized_coordinate_vector[0] +
                U[1, 1, 3] * self.generalized_coordinate_vector[1] +
                U[2, 1, 3] * self.generalized_coordinate_vector[2] +
                U[3, 1, 3] * self.generalized_coordinate_vector[3] +
                U[4, 1, 3] * self.generalized_coordinate_vector[4] +
                U[5, 1, 3] * self.generalized_coordinate_vector[5]],

                [target_z - self.transition_matrix()[2, 3] +
                U[0, 2, 3] * self.generalized_coordinate_vector[0] +
                U[1, 2, 3] * self.generalized_coordinate_vector[1] +
                U[2, 2, 3] * self.generalized_coordinate_vector[2] +
                U[3, 2, 3] * self.generalized_coordinate_vector[3] +
                U[4, 2, 3] * self.generalized_coordinate_vector[4] +
                U[5, 2, 3] * self.generalized_coordinate_vector[5]]
            ])

            result = np.linalg.solve(syst, solve).reshape(6)
            # if np.sum(self.generalized_coordinate_vector - result) ** 2 < eps:
            #     self.generalized_coordinate_vector = result
            #     break

            self.generalized_coordinate_vector = result

            print('\nq: ', self.generalized_coordinate_vector)
            print('EF coords: ', self.get_end_effector_coordinates())
            print(np.linalg.solve(syst, solve).reshape(6))

            count += 1

        pass

    def transition_matrix(self, start_system=0, target_system=6):

        '''
        transition matrix from coordinate starting coordinate system to target system
        '''

        if target_system > self.number_of_joints:
            target_system = self.number_of_joints

        q_twist = self.t_twist + self.generalized_coordinate_vector

        T = np.eye(4)

        for i in range(start_system, target_system):

            A = np.array([
                [cos(q_twist[i]),  -sin(q_twist[i]) * cos(self.a_twist[i]),   sin(q_twist[i]) * sin(self.a_twist[i]),    self.a_shift[i] * cos(q_twist[i])],
                [sin(q_twist[i]),  cos(q_twist[i]) * cos(self.a_twist[i]),    -cos(q_twist[i]) * sin(self.a_twist[i]),   self.a_shift[i] * sin(q_twist[i])],
                [0,                     sin(self.a_twist[i]),                           cos(self.a_twist[i]),                           self.s_shift[i]],
                [0,                     0,                                              0,                                              1]
            ])

            T = T.dot(A)

        return T

    def set_generalized_coordinates(self, generalized_coordinates):
        self.generalized_coordinate_vector = generalized_coordinates

    def get_end_effector_coordinates(self):
        return self.transition_matrix().dot(np.array([0, 0, 0, 1]))[:3]

    def get_end_effector_angles(self):
        return np.array([
            self.transition_matrix()[0, 1],
            self.transition_matrix()[0, 2],
            self.transition_matrix()[1, 2]
        ])


    def get_n_link_coordinates(self, n):
        return self.transition_matrix(target_system=n).dot(np.array([0, 0, 0, 1]))[:3]


if __name__ == '__main__':

    from visualization import draw_robot

    arm = robot_arm()

    print (arm.get_end_effector_coordinates().round(2))
    print (arm.get_end_effector_angles().round(2))
    
    arm.inverse_kinematics()

    print (arm.get_end_effector_coordinates().round(2))
    print (arm.get_end_effector_angles().round(2))
    
    draw_robot(arm)

