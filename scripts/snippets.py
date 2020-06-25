import matplotlib.pyplot as plt
import numpy as np
import json

def plot_forces():
    forces_torques = []
    with open('../data/franka_data_collision.dump') as f_data:
        for line in f_data.readlines():
            data = json.loads(line)
            forces_torques.append(data['O_F_ext_hat_K'])
    np_ft = np.array(forces_torques)

    if False:
        plt.plot(np_ft, label='y = x')
        plt.show()

def plot_torques():
    torques = []
    with open('../data/franka_data_torques.dump') as f_data:
        for line in f_data.readlines():
            torques.append([float(x) for x in line.split(',') if x != '\n'])
    np_t = np.array(torques)

    plt.plot(np_t[:,5], label='y = x')
    plt.show()



def bounding_closest_point():
    box = np.array([
        [0.3, 0.73], 
        [-0.6, 0.1],
        [0, 0.75]
        ])

    pt = np.array([[0.74, -0.4, -0.9]]).T
    close_ax = np.argmin(np.abs(pt - box), axis=1)

    x = pt[0,0] if box[0,0] < pt[0] < box[0,1] else box[0, close_ax[0]]
    y = pt[1,0] if box[1,0] < pt[1] < box[1,1] else box[1, close_ax[1]]
    z = pt[2,0] if box[2,0] < pt[2] < box[2,1] else box[2, close_ax[2]]
    print(x,y,z)

if __name__ == "__main__":
    # plot_forces()
    # bounding_closest_point()

    plot_torques()

    




    

