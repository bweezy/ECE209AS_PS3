import state
import measurement
import numpy as np 

class TwoWheeledRobot:

	def __init__(self, b=85.0, r=20.0, initial_state=state.State(0,0,0), f_s=1):
		self.b = b # Distance between wheels
		self.r = r # Radius of wheels
		self.real_state = initial_state # Recording of real state
		self.estimated_state_mean = initial_state # Recording of estimated state mean
		self.covariance = np.zeros((3,3))  #Initial covariance matrix, set to zero for known initial state
		self.d_t = 1.0/f_s # Time step
		self.Q = np.eye(2) * (np.pi / 6.0) ** 2 # Process noise variance
		self.R = np.eye(2) * (9.375) ** 2 # Measurement noise variance


	# Movement 
	def time_update(self, u):

		# Update the real state - includes noise
		self.real_state_update(u)
		# Update the state estimate - does not include noise
		self.estimated_state_update(u)


	# Updating the real state
	def real_state_update(self, u):

		w_l, w_r = u.get_input()

		b = self.b
		r = self.r
		d_t = self.d_t

		# Add noise to the input
		w_l, w_r = u.get_input()
		w_l = w_l + np.pi / 6.0 * np.random.randn() 
		w_r = w_r + np.pi / 6.0 * np.random.randn()

		x, y, theta = self.real_state.get_state()

		# Update theta
		theta_new = (d_t * r / b * (w_r - w_l) + theta) % (2 * np.pi) 

		# Update x and y given theta and theta new
		if w_r == w_l:
			x = x + r * w_r * d_t * np.cos(theta)
			y = y + r * w_r * d_t * np.sin(theta)
		else:
			R = b / 2.0 * (w_r + w_l)/(w_r - w_l) # Circle radius
			x = x + R * (np.sin(theta_new) - np.sin(theta))
			y = y - R * (np.cos(theta_new) - np.cos(theta))

		self.real_state = state.State(x, y, theta_new)


	# Updating the state estimate
	def estimated_state_update(self, u):

		w_l, w_r = u.get_input()

		b = self.b
		r = self.r
		d_t = self.d_t

		x, y, theta = self.estimated_state_mean.get_state()
		# Update theta using non-noisy input
		theta_new = (d_t * r / b * (w_r - w_l) + theta) % (2 * np.pi) 

		#Update x and y given theta and theta new
		if w_r == w_l:
			x = x + r * w_r * d_t * np.cos(theta)
			y = y + r * w_r * d_t * np.sin(theta)
			df0_dtheta = 0
			df1_dtheta = 0
		else:
			R = b / 2.0 * (w_r + w_l)/(w_r - w_l)
			x = x + R * (np.sin(theta_new) - np.sin(theta))
			y = y - R * (np.cos(theta_new) - np.cos(theta))
			# Derivative of state updates with respect to theta
			df0_dtheta = R * (np.cos(theta_new) - np.cos(theta))
			df1_dtheta = R * ( -1.0 * np.sin(theta_new) + np.sin(theta))

		# Create dynamics jacobian
		F = np.eye(3)
		F[0][2] = df0_dtheta
		F[1][2] = df1_dtheta
		W = self.get_process_noise_jacobian(u)
		self.estimated_state_mean = state.State(x, y, theta_new)
		# Update covariance with dynamics and process noise jacobians
		self.covariance = F.dot(self.covariance).dot(F.T) + W.dot(self.Q).dot(W.T)


	# Calculates the process noise jacobian given the input
	def get_process_noise_jacobian(self, u):

		b = self.b
		r = self.r
		w_l, w_r = u.get_input()
		x, y, theta = self.estimated_state_mean.get_state()

		# See report for derivations
		if w_r == w_l:
			dx_dwl = r * w_l / (2.0 * b) * np.sin(theta)
			dx_dwr = -1.0 * r * w_r / (2.0 * b) * np.sin(theta)
			dy_dwl = r * w_l / (2.0 * b) * np.cos(theta)
			dy_dwr = -1.0 * r * w_r / (2.0 * b) * np.cos(theta)

		else:
			cos = np.cos(r / b * (w_r - w_l) + theta)
			sin = np.sin(r / b * (w_r - w_l) + theta)
			
			dx_dwl = b / r * w_r / (w_r - w_l)**2 * sin - (w_r + w_l) / (2 * (w_r - w_l)) * cos - b / r * w_r / (w_r - w_l)**2 * np.sin(theta) 
			dx_dwr = -1.0 * b / r * w_l / (w_r - w_l)**2 * sin + (w_r + w_l) / (2 * (w_r - w_l)) * cos + b / r * w_l / (w_r - w_l)**2 * np.sin(theta) 

			dy_dwl = b / r * w_r / (w_r - w_l)**2 * cos + (w_r + w_l) / (2 * (w_r - w_l)) * sin - b / r * w_r / (w_r - w_l)**2 * np.cos(theta) 
			dy_dwr = -1.0 * b / r * w_l / (w_r - w_l)**2 * cos - (w_r + w_l) / (2 * (w_r - w_l)) * sin + b / r * w_l / (w_r - w_l)**2 * np.cos(theta)

		dtheta_dwl = -1.0 * r / b
		dtheta_dwr = -1.0 * r / b

		W = np.array([[dx_dwl, dx_dwr], [dy_dwl, dy_dwr], [dtheta_dwl, dtheta_dwr]])
		return W
		

	# Update state and covariance given measurements
	def measurement_update(self):

		x, y, theta = self.real_state.get_state()

		# Calculate rangefinder results given the state
		y_t, front_wall_idx, right_wall_idx = self.measure(self.real_state)
		# Right here we are currently returning which walls the sensors are measuring
		# This helps us use the correct measurement model to compute the Jacobian
		# I think this is technically cheating -> should maybe compute all jacobians and choose one with lowest estimation error?
		H = self.get_observation_jacobian(x, y, theta, front_wall_idx, right_wall_idx)

		# Add noise to the measurement
		y_t = (np.array([y_t.get_measurement()]) + np.random.randn() * (9.375)).T

		# Create prediction given the estimated state
		predicted_y_t, _, _ = self.measure(self.estimated_state_mean)
		predicted_y_t = np.array([predicted_y_t.get_measurement()]).T

	
		sigma_m = self.covariance
		inv_mat = np.linalg.inv(H.dot(sigma_m).dot(H.T) + self.R)
		x_hat = np.array([self.estimated_state_mean.get_state()]).T

		# Update estimate and covariance given observation jacobian
		x_hat += sigma_m.dot(H.T).dot(inv_mat).dot((y_t - predicted_y_t))
		x_hat = np.squeeze(x_hat)
		self.covariance = sigma_m - sigma_m.dot(H.T).dot(inv_mat).dot(H).dot(sigma_m)
		self.estimated_state_mean = state.State(x_hat[0], x_hat[1], x_hat[2])


	# Returns the observation jacobian given the state and which wall each sensor is reading
	def get_observation_jacobian(self, x, y, theta, front_wall_idx, right_wall_idx):

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

		# Can save a lot of computation time if we just compute all cos values up front
		# dd<a><b>_d<c> = partial derivative of rangefinder facing <a> and reading wall <b> with respect to <c>
		ddf1_dx = -1.0 / np.cos(theta)
		ddf2_dx = 0
		ddf3_dx = 1.0 / np.cos(theta - np.pi)
		ddf4_dx = 0
		ddf_dx = [ddf1_dx, ddf2_dx, ddf3_dx, ddf4_dx]


		ddf1_dy = 0
		ddf2_dy = -1.0 / np.cos(theta - np.pi / 2.0)
		ddf3_dy = 0
		ddf4_dy = 1.0 / np.cos(theta - np.pi * 3.0 / 2.0)
		ddf_dy = [ddf1_dy, ddf2_dy, ddf3_dy, ddf4_dy]

		ddf1_dtheta = (500 - x) * np.tan(theta) / np.cos(theta)
		ddf2_dtheta = (750 - y) * np.tan(theta - np.pi / 2.0) / np.cos(theta - np.pi / 2.0)
		ddf3_dtheta = x * np.tan(theta - np.pi) / np.cos(theta - np.pi)
		ddf4_dtheta = y * np.tan(theta - np.pi * 3.0 / 2.0) / np.cos(theta - np.pi * 3.0 / 2.0)
		ddf_dtheta = [ddf1_dtheta, ddf2_dtheta, ddf3_dtheta, ddf4_dtheta]

		ddr1_dx = -1.0 / np.cos(theta - np.pi / 2.0)
		ddr2_dx = 0
		ddr3_dx = 1.0 / np.cos(theta - np.pi * 3.0 / 2.0)
		ddr4_dx = 0
		ddr_dx = [ddr1_dx, ddr2_dx, ddr3_dx, ddr4_dx]

		ddr1_dy = 0
		ddr2_dy = -1.0 / np.cos(theta - np.pi)
		ddr3_dy = 0
		ddr4_dy = 1.0 / np.cos(theta)
		ddr_dy = [ddr1_dy, ddr2_dy, ddr3_dy, ddr4_dy]

		ddr1_dtheta = (500 - x) * np.tan(theta - np.pi / 2.0) / np.cos(theta - np.pi / 2.0)
		ddr2_dtheta = (750 - y) * np.tan(theta - np.pi) / np.cos(theta - np.pi)
		ddr3_dtheta = x * np.tan(theta - np.pi * 3.0 / 2.0) / np.cos(theta - np.pi * 3.0 / 2.0)
		ddr4_dtheta = y * np.tan(theta) / np.cos(theta)
		ddr_dtheta = [ddr1_dtheta, ddr2_dtheta, ddr3_dtheta, ddr4_dtheta]

		# Jacobian
		H = np.array([[ddf_dx[front_wall_idx - 1], ddf_dy[front_wall_idx - 1], ddf_dtheta[front_wall_idx - 1]],
		[ddr_dx[right_wall_idx - 1], ddr_dy[right_wall_idx - 1], ddr_dtheta[right_wall_idx - 1]]])

		return H



	# Given a state, return the rangefinder results
	def measure(self, state_in):

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

		x, y, theta = state_in.get_state()

		# Calculate the right rangefinder results for each wall
		dr1 = (500 - x) / np.cos(theta - np.pi / 2.0)
		dr2 = (750 - y) / np.cos(theta - np.pi)
		dr3 = x / np.cos(theta - np.pi * 3.0 / 2.0)
		dr4 = y / np.cos(theta)

		dr = [dr1, dr2, dr3, dr4]
		min_pos_dr = float("inf")
		right_wall_idx = None

		# The wall the right rangefinder should read is the smallest positive number of the above values
		for i in np.arange(len(dr)):
			if dr[i] > 0 and dr[i] < min_pos_dr:
				min_pos_dr = dr[i]
				right_wall_idx = i + 1

		# Calculate the right rangefinder results for each wall
		df1 = (500 - x) / np.cos(theta)
		df2 = (750 - y) / np.cos(theta - np.pi / 2.0)
		df3 = (x) / np.cos(theta - np.pi)
		df4 = (y) / np.cos(theta - np.pi * 3.0 / 2.0)

		df = [df1, df2, df3, df4]
		min_pos_df = float("inf")
		front_wall_idx = None
		# The wall the front rangefinder should read is the smallest positive number of the above values
		for i in np.arange(len(df)):
			if df[i] > 0 and df[i] < min_pos_df:
				min_pos_df = df[i]
				front_wall_idx = i + 1

		return measurement.Measurement(min_pos_df, min_pos_dr), front_wall_idx, right_wall_idx