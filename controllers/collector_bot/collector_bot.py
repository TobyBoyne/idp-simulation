"""collector_bot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
TIME_STEP = 32
MAX_SPEED = 6.28

# get the time step of the current world.

# setup motors
# left_motor = robot.getDevice('left_motor')
# left_motor.setPosition(float('inf'))

# right_motor = robot.getDevice('right_motor')
# right_motor.setPosition(float('inf'))

# setup sensors
# ds = robot.getDevice('distance_sensor')
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)


while robot.step(TIME_STEP) != -1:
    # READ SENSORS
    # left_motor.setVelocity(0.5 * MAX_SPEED)
    pos = gps.getValues()  
    print(pos)
    # PROCESS

    # ACTUATORS
    #  motor.setPosition(10.0)