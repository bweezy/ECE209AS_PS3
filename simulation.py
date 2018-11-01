from third_party_functions import plot_covariance_ellipse
import state
import twowheeledrobot as twr
import Input
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)
                
def run_simulation(init_state):

    robot = twr.TwoWheeledRobot(initial_state=init_state)
    u = [1, 2]

    plt.figure()
    
    trajectory = []
    for i in range(27):
        print i

        robot.time_update(Input.Input(u[0], u[1]))

        real_state = robot.real_state.get_state_degrees()
        trajectory.append(real_state)

        estimated_state_mean = robot.estimated_state_mean.get_state_degrees()
        covariance = robot.covariance

        # import pdb; pdb.set_trace()

        # plot_covariance_ellipse((estimated_state_mean[0], estimated_state_mean[1]), 
        #     covariance[0:2, 0:2], std=6, facecolor='k', alpha=0.3)

        robot.measurement_update()
        estimated_state_mean = robot.estimated_state_mean.get_state_degrees()
        covariance = robot.covariance

        # plot_covariance_ellipse((estimated_state_mean[0], estimated_state_mean[1]), 
        #     covariance[0:2, 0:2], std=6, facecolor='g', alpha=0.8)

    trajectory = np.array(trajectory)
    plt.plot(trajectory[:, 0], trajectory[:,1], color='k', lw=2)
    plt.axis('equal')
    # if ylim is not None: plt.ylim(*ylim)
    plt.show()


if __name__ == "__main__":
    initial_state = state.State(250,375, np.pi / 2.0)
    run_simulation(initial_state)