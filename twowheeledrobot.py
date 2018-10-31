import state
import action
import measurement


class TwoWheeledRobot:

	def __init__(self, b=85, r=20, initial_state=state.State(0,0,0)):
		self.b = b # Distance between wheels
		self.r = r # Radius of wheels

	