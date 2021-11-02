import numpy as np
import matplotlib.pyplot as plt

def draw_robot(arm):

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_zlim(0, 300)

    data = np.array([[0, 0, 0]])

    for i in range(1, arm.number_of_joints):
        data = np.vstack((data, arm.get_n_link_coordinates(i).round(2)))
    data = np.vstack((data, arm.get_end_effector_coordinates().round(2)))
    
    plt.plot(data[:,0], data[:,1], data[:,2], '--ko')
    plt.show()
