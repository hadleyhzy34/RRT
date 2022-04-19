import numpy as np
from matplotlib import pyplot as plt

if __name__ == '__main__':
    data = np.loadtxt("/home/hadley/development/c-robotics/path_planning/rrt/traj.csv", delimiter=",")
    import ipdb;ipdb.set_trace()
    print(data)
    
    obstacle_list = np.array([[5, 1, np.sqrt(2)/2],
                              [3, 6, np.sqrt(2)/2],
                              [3, 8, np.sqrt(2)/2],
                              [1, 1, np.sqrt(2)/2],
                              [3, 5, np.sqrt(2)/2],
                              [9, 5, np.sqrt(2)/2]])


    for (ox, oy, size) in obstacle_list:
        plt.plot(ox, oy, "sk", ms=20*size)

    plt.plot(0., 0., "^r")
    plt.plot(6., 8., "^b")
    
    plt.axis([-2., 10., -2., 10.])
    plt.plot(data[:,0],data[:,1])

    plt.grid(True)
    plt.show()
