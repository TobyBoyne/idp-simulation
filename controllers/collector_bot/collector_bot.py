"""collector_bot controller."""

import time
import math

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio

from controller import Robot, DistanceSensor


TIME_STEP = 16
MAX_SPEED = 6.28

# parameters based on the colour of the robot
ROBOT_PARAMS = {
    'red_bot': {'colour': 'red', 'channel': 1}
}




class Collector(Robot):
    def __init__(self):
        super().__init__()
        
        # setup sensors
        self.gps = self.getDevice('gps')
        self.gps.enable(TIME_STEP)
        
        self.compass = self.getDevice('compass')
        self.compass.enable(TIME_STEP)
        
        self.dist_sensor = self.getDevice('distance1')
        self.dist_sensor.enable(TIME_STEP)
        
        self.name = self.getName()
        
        # setup motors
        self.leftmotor = self.getDevice('wheel1')
        self.rightmotor = self.getDevice('wheel2')
        
        self.leftmotor.setPosition(float('inf'))
        self.rightmotor.setPosition(float('inf'))
        self.leftmotor.setVelocity(0.0)
        self.rightmotor.setVelocity(0.0)
        
        self.radio = Radio(channel=1)
        
        self.commands = {
            'SPN': self._spin,
            'MOV': self._move,
        }
        
        self.data = []
        
        
    def runCommand(self, cmd, *values):
        if cmd in self.commands:
            self.commands[cmd](*values)
        
    def _spin(self, *args):
    
        # Set spinning
        self.leftmotor.setVelocity(0.05*MAX_SPEED)
        self.rightmotor.setVelocity(-0.05*MAX_SPEED)
        
        # Record spatial information
        pos = self.gps.getValues()
        d = self.dist_sensor.getValue()
        heading = self._getBearing()
        
        # Process dist sensor measurement
        d = 0.7611*(d**(-0.9313))-0.1252
        
        # Find Target Coords
        pos_d = [pos[0] - 0.01, pos[2] + 0.12]
        pos_t = [pos_d[0] - d*math.cos(heading), pos_d[1] - d*math.sin(heading)]
        
        # Store for evaluation
        self.data.append(pos_t)
        with open('listfile.txt', 'w') as filehandle:
            for listitem in self.data:
                filehandle.write('%s\n' % listitem)
        
        # print('Spinning')
        
    def _move(self, *args):
        """Move to a point, with some basic collision avoidance along the way"""
        print('Moving')
        
        
    def _getBearing(self):
        """Returns the current bearing of the robot in radians"""
        north = self.compass.getValues()
        rad = math.atan2(north[0], north[2])
        return rad
    
    def run(self):
        while robot.step(TIME_STEP) != -1:
            
            msg = self.radio.receive()
            if msg is not None:
                self.runCommand(*msg)
            
            
robot = Collector()            
robot.run()