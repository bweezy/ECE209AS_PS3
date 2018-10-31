import twowheeledrobot as twr
import Input

robot = twr.TwoWheeledRobot()

u = Input.Input(1,1.1)

robot.time_update(u)

print robot.real_state.get_state()
print robot.covariance
