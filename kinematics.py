import numpy as np
from math import sin, cos, pi

def transition_matrix(generalized_coordinate_vector, start_system=0, target_system=6):
    
    """
    transition matrix from coordinate starting coordinate system to target system
    """

    #sizes
    h1 = 50
    l34 = 300
    l56 = 200
    l1 = 40
    h2 = 150

    # Denavit–Hartenberg parameters
    t_twist = np.array([0,     pi/2,   0,      0,      0,      pi/2])
    s_shift = np.array([h1,    0,      0,      l34,    0,      l56])
    a_shift = np.array([l1,    h2,     0,      0,      0,      0])
    a_twist = np.array([pi/2,  0,      pi/2,   -pi/2,  pi/2,   0])

    t_twist = t_twist + generalized_coordinate_vector

    T = np.eye(4)

    for i in range(start_system, target_system):

        A = np.array([
            [cos(t_twist[i]),   -sin(t_twist[i]) * cos(a_twist[i]), sin(t_twist[i]) * sin(a_twist[i]),  a_shift[i] * cos(t_twist[i])],
            [sin(t_twist[i]),   cos(t_twist[i]) * cos(a_twist[i]),  -cos(t_twist[i]) * sin(a_twist[i]), a_shift[i] * sin(t_twist[i])],
            [0,                 sin(a_twist[i]),                    cos(a_twist[i]),                    s_shift[i]],
            [0,                 0,                                  0,                                  1]
        ])

        T = T.dot(A)

    return T

generalized_coordinate_vector = np.array([0, 0, 0, 0, 0, 0])

if __name__ == '__main__':

    test_vector = np.array ([0, 0, 0, 0])
    print(transition_matrix(generalized_coordinate_vector).round(2))
