import numpy as np

class State:

	def __init__(self, pos_x=0, pos_y=0, theta=0):
		self.pos_x = pos_x
		self.pos_y = pos_y
		self.theta = theta

	def get_state(self):
		return self.pos_x, self.pos_y, self.theta

	def get_state_degrees(self):
		return self.pos_x, self.pos_y, self.theta * 180.0 / np.pi