from third_party_functions import plot_covariance_ellipse
import state
import twowheeledrobot as twr
import Input
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

np.random.seed(1)

env_W = 500
env_L = 750
                
def run_simulation(init_state):

    robot = twr.TwoWheeledRobot(initial_state=init_state)
    u = [1, 2]

    fig, ax = plt.subplots(1)
    
    trajectory = []
    trajectory.append(robot.real_state.get_state_degrees())
    estimated_state_mean = robot.estimated_state_mean.get_state_degrees()
    covariance = robot.covariance
    plot_covariance_ellipse((estimated_state_mean[0], estimated_state_mean[1]), 
            covariance[0:2, 0:2], facecolor='k', alpha=0.3)
    for i in range(15):
        robot.time_update(Input.Input(u[0], u[1]))

        real_state = robot.real_state.get_state_degrees()
        trajectory.append(real_state)

        estimated_state_mean = robot.estimated_state_mean.get_state_degrees()
        covariance = robot.covariance

        # # import pdb; pdb.set_trace()
        # # print i, " iteration: "
        # # print "  ", estimated_state_mean[0], estimated_state_mean[1]

        plot_covariance_ellipse((estimated_state_mean[0], estimated_state_mean[1]), 
            covariance[0:2, 0:2], facecolor='k', alpha=0.3)

        robot.measurement_update()
        estimated_state_mean = robot.estimated_state_mean.get_state_degrees()
        covariance = robot.covariance

        # print "  ", estimated_state_mean[0], estimated_state_mean[1]
        plot_covariance_ellipse((estimated_state_mean[0], estimated_state_mean[1]), 
            covariance[0:2, 0:2], facecolor='g', alpha=0.8)

    trajectory = np.array(trajectory)
    plt.plot(trajectory[:, 0], trajectory[:,1], color='k', lw=2)
    rect = patches.Rectangle((0,0),env_W,env_L,linewidth=1,edgecolor='r',facecolor='none')
    plt.ylim(-10, env_L+10)
    plt.xlim(-10, env_W+10)
    ax.add_patch(rect)
    # if ylim is not None: plt.ylim(*ylim)
    plt.show()


if __name__ == "__main__":
    initial_state = state.State(400,375, np.pi / 2.0)
    # initial_state = state.State(100,100, np.pi / 2.0)
    run_simulation(initial_state)