"""collector_bot controller."""

import time
import math

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio

from controller import Robot


TIME_STEP = 32
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
        
        self.name = self.getName()
        
        # setup motors
        
        self.radio = Radio(channel=1)
        
        self.commands = {
            'SPN': self._spin,
            'MOV': self._move,
        }
        
        
    def runCommand(self, cmd, *values):
        if cmd in self.commands:
            self.commands[cmd](*values)
        
    def _spin(self, *args):
        """Spin one full rotation while reading from distance sensor"""
        print('Spinning')
        
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
            pos = self.gps.getValues()
            
            heading = self._getBearing()
            print(heading)
            
            msg = self.radio.receive()
            if msg is not None:
                self.runCommand(*msg)
            
            if time.perf_counter() % 1 < 0.1:
                msg = ('GPS', pos[0], pos[2])
                self.radio.send(*msg)
            
            
robot = Collector()            
robot.run()