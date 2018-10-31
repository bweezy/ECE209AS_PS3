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
		theta_new_rad = (d_t * r / b * (w_r - w_l) + theta_rad) % (2 * np.pi) 

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


	# This is super Brute Force, should be a better way
	def measurement_update(self):

		x, y, theta_deg = self.real_state.get_state()
		theta_rad = theta_deg * np.pi / 180.0

		y_t, front_wall_idx, right_wall_idx = self.measure()
		# Right here we are currently returning which walls the sensors are measuring
		# This helps us use the correct measurement model to compute the Jacobian
		# I think this is technically cheating -> should maybe compute all jacobians and choose one with lowest estimation error?

		df, dr = y_t.get_measurement()

		# Derivative of df with respect to x assuming that f is seeing the front wall
		# Can save a lot of computation time if we just compute all cos values up front
		ddf1_dx = -1.0 / np.cos(theta_rad)
		ddf2_dx = 0
		ddf3_dx = 1.0 / np.cos(theta_rad - np.pi)
		ddf4_dx = 0
		ddf_dx = [ddf1_dx, ddf2_dx, ddf3_dx, ddf4_dx]


		ddf1_dy = 0
		ddf2_dy = -1.0 / np.cos(theta_rad - np.pi / 2.0)
		ddf3_dy = 0
		ddf4_dy = 1.0 / np.cos(theta_rad - np.pi * 3.0 / 2.0)
		ddf_dy = [ddf1_dy, ddf2_dy, ddf3_dy, ddf4_dy]

		ddf1_dtheta = (500 - x) * np.tan(theta_rad) / np.cos(theta_rad)
		ddf2_dtheta = (750 - y) * np.tan(theta_rad - np.pi / 2.0) / np.cos(theta_rad - np.pi / 2.0)
		ddf3_dtheta = x * np.tan(theta_rad - np.pi) / np.cos(theta_rad - np.pi)
		ddf4_dtheta = y * np.tan(theta_rad - np.pi * 3.0 / 2.0) / np.cos(theta_rad - np.pi * 3.0 / 2.0)
		ddf_dtheta = [ddf1_dtheta, ddf2_dtheta, ddf3_dtheta, ddf4_dtheta]

		ddr1_dx = -1.0 / np.cos(theta_rad - np.pi / 2.0)
		ddr2_dx = 0
		ddr3_dx = 1.0 / np.cos(theta_rad - np.pi * 3.0 / 2.0)
		ddr4_dx = 0
		ddr_dx = [ddr1_dx, ddr2_dx, ddr3_dx, ddr4_dx]

		ddr1_dy = 0
		ddr2_dy = -1.0 / np.cos(theta_rad - np.pi)
		ddr3_dy = 0
		ddr4_dy = 1.0 / np.cos(theta_rad)
		ddr_dy = [ddr1_dy, ddr2_dy, ddr3_dy, ddr4_dy]

		ddr1_dtheta = (500 - x) * np.tan(theta_rad - np.pi / 2.0) / np.cos(theta_rad - np.pi / 2.0)
		ddr2_dtheta = (750 - y) * np.tan(theta_rad - np.pi) / np.cos(theta_rad - np.pi)
		ddr3_dtheta = x * np.tan(theta_rad - np.pi * 3.0 / 2.0) / np.cos(theta_rad - np.pi * 3.0 / 2.0)
		ddr4_dtheta = y * np.tan(theta_rad) / np.cos(theta_rad)
		ddr_dtheta = [ddr1_dtheta, ddr2_dtheta, ddr3_dtheta, ddr4_dtheta]

		# Jacobian
		H = np.array([[ddf_dx[front_wall_idx - 1], ddf_dy[front_wall_idx - 1], ddf_dtheta[front_wall_idx - 1]],
		[ddr_dx[right_wall_idx - 1], ddr_dy[right_wall_idx - 1], ddr_dtheta[right_wall_idx - 1]]])


		print H



	def measure(self):

		# Define walls:
		#             2 (top)
		#         ----------------
		#         |              |
		#         |              |
		# 3(left) |              | 1 (right)
		#         |              |
		#         |              |
		#         |              |
		#         ---------------- 
		#            4 (bottom)

		x, y, theta_deg = self.real_state.get_state()
		theta_rad = theta_deg * np.pi / 180.0

		dr1 = (500 - x) / np.cos(theta_rad - np.pi / 2.0)
		dr2 = (750 - y) / np.cos(theta_rad - np.pi)
		dr3 = x / np.cos(theta_rad - np.pi * 3.0 / 2.0)
		dr4 = y / np.cos(theta_rad)

		dr = [dr1, dr2, dr3, dr4]
		min_pos_dr = float("inf")
		right_wall_idx = None

		for i in np.arange(len(dr)):
			if dr[i] > 0 and dr[i] < min_pos_dr:
				min_pos_dr = dr[i]
				right_wall_idx = i + 1

		df1 = (500 - x) / np.cos(theta_rad)
		df2 = (750 - y) / np.cos(theta_rad - np.pi / 2.0)
		df3 = (x) / np.cos(theta_rad - np.pi)
		df4 = (y) / np.cos(theta_rad - np.pi * 3.0 / 2.0)

		df = [df1, df2, df3, df4]
		min_pos_df = float("inf")
		front_wall_idx = None

		for i in np.arange(len(df)):
			if df[i] > 0 and df[i] < min_pos_df:
				min_pos_df = df[i]
				front_wall_idx = i + 1

		'''
		print 'df ',
		print min_pos_df
		print 'front wall idx ',
		print front_wall_idx

		print 'dr ',
		print min_pos_dr
		print 'right wall idx ',
		print right_wall_idx
		'''

		return measurement.Measurement(min_pos_df, min_pos_dr), front_wall_idx, right_wall_idx