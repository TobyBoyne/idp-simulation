"""Code to control each of the robots"""


from controller import Robot

TIME_STEP = 32
MAX_VELOCITY = 6.28

BOT_TYPE = {
	'red_bot': 0,
	'blue_bot': 1
}


class Collector(Robot):
	def __init__(self):

		super().__init__()
		self.time_step = TIME_STEP
		self.bot_type = BOT_TYPE.get(self.getName(), None)

		# self.distance_sensor = self.getDistanceSensor('distance_sensor')
		# self.distance_sensor.enable(self.time_step)

		self.left_motor = self.getDevice('left wheel motor')
		self.left_motor.setPosition(float('inf'))

	def run(self):
		while self.step(self.time_step) != -1:
			self.left_motor.setVelocity(0.5 * MAX_VELOCITY)
