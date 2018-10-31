import twowheeledrobot as twr
import state
import Input
import numpy as np

robot = twr.TwoWheeledRobot(initial_state=state.State(250,375, np.pi / 2.0))


print 'initial_state '
print robot.real_state.get_state_degrees()
print

u = Input.Input(1.1,1)
robot.time_update(u)

print '--movement update--'
print 'real_state'
print robot.real_state.get_state_degrees()
print

print 'estimated'
print robot.estimated_state_mean.get_state_degrees()
print robot.covariance
print


robot.measurement_update()
print '--measurement_update--'
print 'estimated'
print robot.estimated_state_mean.get_state_degrees()
print robot.covariance
print