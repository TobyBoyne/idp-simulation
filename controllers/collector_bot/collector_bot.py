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

W = 512
def displayNormalise(v):
    """Normalise a postion array for the display"""
    norm = ((v + 1.2) / 2.4) * W
    clipped = np.clip(norm, 0, W)
    # convert to list to use base numpy type int
    return clipped.astype(int).tolist()


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
        
        # display useful for debugging
        self.display = self.getDevice('display')
        
    def clearQueue(self):
        self._drive(0)
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
            self.radio.send('DNE', 0., 0.)
                
    def _move(self, *args):
        """Move to a point, with some basic collision avoidance along the way"""
        
        pos = self._getPos()
        target = np.array(args)
        v_targ = (target - pos) / np.linalg.norm(target - pos)
        v_head = self._getBearing(as_vector=True)
        
        # compute cross product(v_targ, v_head) 
        # sign shows which direction to turn
        cross = v_head[0] * v_targ[1] - v_head[1] * v_targ[0]
        
        # dot product determines spin speed
        # large dot product -> closely alligned -> small speed
        dot = np.clip(np.dot(v_head, v_targ), 0, 0.8)
        base_speed = 0.5
        
        v_left =  base_speed - (1 - dot if cross < 0 else 0)
        v_right = base_speed - (1 - dot if cross > 0 else 0)
        self._wheelMotors(v_left, v_right)
                
        
        if np.linalg.norm(target - pos) < 0.1:
            self.clearQueue()
            self.radio.send('DNE', 0., 0.)
        
        
        
        # draw to the display (for debugging)
        w = self.display.getWidth()
        self.display.setFont('Lucida Console', 20, True)
        
        self.display.setColor(0)
        self.display.fillPolygon([0, 0, w, w], [0, w, w, 0])
       
        x1, y1 = displayNormalise(pos)
        x2, y2 = displayNormalise(pos + v_targ * 0.1)
        x3, y3 = displayNormalise(pos + v_head * 0.1)
        
        x_t, y_t = displayNormalise(target)
        red_home = np.array([[0.8, 1.2, 1.2, 0.8], [0.8, 0.8, 1.2, 1.2]])
        blue_home = (red_home.T + np.array([0., -2.,])).T
        # plot car and home
        self.display.setColor(int('0xff0000', 16))
        self.display.fillOval(x1, y1, 3, 3)
        self.display.fillPolygon(*displayNormalise(red_home))
        
        self.display.setColor(int('0xffffff', 16))
        self.display.drawText('Target vector', 10, 10)
        self.display.drawLine(x1, y1, x2, y2)
        self.display.fillOval(x_t, y_t, 3, 3)
        self.display.drawText('left' if cross<0 else 'right', 10, 450)
        
        self.display.setColor(int('0x5555ff', 16))
        self.display.drawText('Heading vector', 10, 40)
        self.display.drawLine(x1, y1, x3, y3)
        self.display.fillPolygon(*displayNormalise(blue_home))
                
    def _wheelMotors(self, v_left, v_right):
        # wheels might be backwards?
        self.leftmotor.setVelocity(v_left * MAX_SPEED)
        self.rightmotor.setVelocity(v_right * MAX_SPEED)
        
    def _drive(self, v):
        self._wheelMotors(v, v)
        
    def _spin(self, v):
        self._wheelMotors(v, -v)

        
    def _getPos(self):
        """Returns current position of the robot as a 2-length vector"""
        pos_lst = self.gps.getValues()
        return np.array([pos_lst[0], pos_lst[2]])
        
    def _getBearing(self, as_vector=False):
        """Returns the current bearing of the robot in radians
        If as_vector, return the unit vector in the direction of the heading"""
        north = self.compass.getValues()
        if as_vector:
            # v = np.array([north[0], north[2]])
            # return v / np.linalg.norm(v)
            v = np.array([north[2], north[0]])
            return v / np.linalg.norm(v)


        rad = np.arctan2(north[0], north[2])
        return rad
    
    def run(self):
        while self.step(TIME_STEP) != -1:
            
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