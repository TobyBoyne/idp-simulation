"""collector_bot controller."""

import time
import math
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio

from controller import Robot, DistanceSensor


TIME_STEP = 16
MAX_SPEED = 6.28

DIST_TO_SENSOR = 0.12

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
            'SCN': self._scan,
            'MOV': self._move,
        }
        
        self.data = []
        
        self.clearQueue()
        
        
    def clearQueue(self):
        
        self.cur_command = None
        self.cur_values = []
        
        if self.data:
            np.save('listnp.npy', np.array(self.data)) 
            # self.radio.send(self.data)
            self.data = []
        
             
    def runCommand(self):
        if self.cur_command is not None:
            self.cur_command(*self.cur_values)
        
    
    def _scan(self, *args):
        """Scan the environment with a full spin recording from the distance sensor"""
        
        # Set spinning
        self._spin(0.05)
        
        # Record spatial information
        pos = self._getPos()
        d = self.dist_sensor.getValue()
        heading_vec = self._getBearing(as_vector=True)
        # Process dist sensor measurement
        d = 0.7611 * (d**(-0.9313)) - 0.1252
        
        # Find Target Coords
        pos_d = pos - DIST_TO_SENSOR * heading_vec
        pos_t = pos_d - d * heading_vec
                
        # Store for evaluation
        self.data.append(pos_t)
        
        # Completeness Test
        if len(self.data) > 2000:
            self.clearQueue()
            self._drive(0)
                
        
    def _move(self, *args):
        """Move to a point, with some basic collision avoidance along the way"""
        
        pos = self._getPos()
        # target = np.array(*args)
        self._drive(0.5)
        
        
    def _drive(self, v):
        self.leftmotor.setVelocity(v*MAX_SPEED)
        self.rightmotor.setVelocity(v*MAX_SPEED)
        
        
    def _spin(self, v):
        self.leftmotor.setVelocity(v*MAX_SPEED)
        self.rightmotor.setVelocity(-v*MAX_SPEED)

        
    def _getPos(self):
        """Returns current position of the robot as a 2-length vector"""
        pos_lst = self.gps.getValues()
        return np.array([pos_lst[0], pos_lst[2]])
        
    def _getBearing(self, as_vector=False):
        """Returns the current bearing of the robot in radians
        If as_vector, return the unit vector in the direction of the heading"""
        north = self.compass.getValues()
        if as_vector:
            v = np.array([north[0], north[2]])
            return v / np.linalg.norm(v)

        rad = np.arctan2(north[0], north[2])
        return rad
    
    def run(self):
        while robot.step(TIME_STEP) != -1:
            
            # receive message
            msg = self.radio.receive()
            if msg is not None:
                print(msg)
                cmd, *values = msg
                self.cur_command = self.commands.get(cmd, None)
                self.cur_values = values
                              
                
            # run the last command that was given
            self.runCommand()
            
robot = Collector()            
robot.run()