import twowheeledrobot as twr
import state
import Input
import numpy as np

robot = twr.TwoWheeledRobot(initial_state=state.State(250,375, np.pi / 2.0))

u = Input.Input(1.1,1)

robot.time_update(u)

#print robot.real_state.get_state_degrees()

print robot.estimated_state_mean.get_state_degrees()
#print robot.covariance

robot.measurement_update()