class Measurement:

	def __init__(self, d_f=0, d_r=0, degrees=0):
		self.d_f = d_f
		self.d_r = d_r
		self.degrees = degrees

	def get_measurement(self):
		return self.d_f, self.d_r, self.degrees