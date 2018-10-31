import state
import measurement
import numpy as np 


class TwoWheeledRobot:

	def __init__(self, b=85.0, r=20.0, initial_state=state.State(0,0,0), f_s=1):
		self.b = b # Distance between wheels
		self.r = r # Radius of wheels
		self.real_state = initial_state
		self.estimated_state_mean = initial_state
		self.covariance = np.eye(3) 
		self.d_t = 1.0/f_s


	def time_update(self, u):

		x, y, theta_deg = self.real_state.get_state()
		w_l, w_r = u.get_input()

		b = self.b
		r = self.r
		d_t = self.d_t

		theta_rad = theta_deg * np.pi / 180.0
		theta_new_rad = d_t * r / b * (w_r - w_l) + theta_rad 

		if w_r == w_l:
			x = x + r * w_r * d_t
			y = y + r * w_r * d_t
			df0_dtheta = 0
			df1_dtheta = 0
		else:
			R = b / 2.0 * (w_r - w_l)/(w_r - w_l)
			x = x + R * (np.sin(theta_new_rad) - np.sin(theta_rad))
			y = y - R * (np.cos(theta_new_rad) - np.cos(theta_rad))

			df0_dtheta = R * (np.cos(theta_new_rad) - np.cos(theta_rad))
			df1_dtheta = R * ( -1.0 * np.sin(theta_new_rad) + np.sin(theta_rad))

		theta_deg = theta_new_rad * 180.0 / np.pi
		self.real_state = state.State(x, y, theta_deg)
		self.estimated_state_mean = state.State(x, y, theta_deg)

		F = np.eye(3)
		F[0][2] = df0_dtheta
		F[1][2] = df1_dtheta

		self.covariance = F.dot(self.covariance).dot(F.T)

