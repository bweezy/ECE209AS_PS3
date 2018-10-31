import twowheeledrobot as twr
import state
import Input
import numpy as np

robot = twr.TwoWheeledRobot(initial_state=state.State(250,375, np.pi / 2.0))

'''
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
'''

while(True):
	w_l = input("Input w_l: ")
	w_r = input("Input w_r: ")

	robot.time_update(Input.Input(w_l, w_r))

	real_state = np.array([robot.real_state.get_state_degrees()])
	estimated_state_mean = np.array([robot.estimated_state_mean.get_state_degrees()])
	error = real_state - estimated_state_mean

	print '--movement update--'
	print 'error: ',
	print error

	robot.measurement_update()
	real_state = np.array([robot.real_state.get_state_degrees()])
	estimated_state_mean = np.array([robot.estimated_state_mean.get_state_degrees()])
	error = real_state - estimated_state_mean
	print '--measurement_update--'
	print 'error: ',
	print error



