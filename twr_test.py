import twowheeledrobot as twr
import state
import Input

robot = twr.TwoWheeledRobot(initial_state=state.State(250,375,90))

#u = Input.Input(1,1.1)

#robot.time_update(u)

#print robot.real_state.get_state()
#print robot.covariance

robot.measurement_update()