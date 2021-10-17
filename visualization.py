import numpy as np
import matplotlib.pyplot as plt

def draw_robot(arm):

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    data = np.array([[0, 0, 0]])

    for i in range(1, 7):

        data = np.vstack((data, arm.get_n_link_coordinates(i).round(2)))
    
    plt.plot(data[:,0], data[:,1], data[:,2], '--ko')

    plt.show()
