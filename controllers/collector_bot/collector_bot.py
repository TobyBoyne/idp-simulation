"""collector_bot controller."""

import time
import math
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio
from display import MapDisplay

from controller import Robot

TIME_STEP = 16
MAX_SPEED = 3.14

DIST_TO_SENSOR = 0

# parameters based on the colour of the robot
ROBOT_PARAMS = {
    'red_bot': {'colour': 'red', 'channel': 1}
}

        
def pointInsideWalls(p):
    x, z = p
    # L is slightly less than the actual width to account for noise in distance sensor
    L = 1.16
    # also ignore any points inside the homes
    inside_walls = -L <= x <= L and -L <= z <= L
    in_red_home = 0.8 <= x <= 1.2 and 0.8 <= z <= 1.2
    in_blue_home = 0.8 <= x <= 1.2 and -0.8 >= z >= -1.2
    return inside_walls and not in_red_home and not in_blue_home

def findClusters(data_lst, pos):
    """Find the clusters within the scanned datapoints"""
    data = np.array(data_lst)
    centroids = data[0, None]
    min_R = 0.1

    # if there are few datapoints, break out early
    if data.shape[0] < 2:
        return np.array([])

    for _ in range(8):
        dists = np.linalg.norm(data[:, None, :] - centroids[None, :, :], axis=-1)
        potentials = (1 / dists).sum(axis=1)

        new_c_idx = np.argmin(potentials)

        if np.min(dists[new_c_idx]) < min_R:
            # if this is close to an existing centroid, stop finding centroids
            break

        centroids = np.concatenate([centroids, data[new_c_idx, None]], axis=0)


    # run a single k-means to find the centroid of each cluster
    k = centroids.shape[0]
    dists = np.linalg.norm(data[:, None, :] - centroids[None, :, :], axis=-1)
    closest_centroid = np.argmin(dists, axis=-1)
    
    A = 0.05 * 1.1222 / 2
    for n in range(k):
        new_centroid = data[closest_centroid == n].mean(axis=0)
        # move centroid away from the robot, as the outside of the block is scanned
        # avg radius of square is A
        v = (new_centroid - pos) / np.linalg.norm(pos - new_centroid)
        new_centroid += v * A
        centroids[n] = new_centroid

    return centroids


class Collector(Robot):
    def __init__(self):
        super().__init__()
        
        # setup sensors
        self.gps = self.getDevice('gps')
        self.gps.enable(TIME_STEP)
        
        self.compass = self.getDevice('compass')
        self.compass.enable(TIME_STEP)
        
        self.dist_sensor = self.getDevice('distance_sensor')
        self.dist_sensor.enable(TIME_STEP)
        
        self.camera = self.getDevice('camera')
        self.camera.enable(TIME_STEP)
        
        self.name = self.getName()
        
        # setup motors
        self.leftmotor = self.getDevice('lwheel_motor')
        self.rightmotor = self.getDevice('rwheel_motor')
        
        self.larmmotor = self.getDevice('larm_motor')
        self.rarmmotor = self.getDevice('rarm_motor')
        
        self.larmmotor_pos = self.getDevice('larm_position')
        self.rarmmotor_pos = self.getDevice('rarm_position')
        self.larmmotor_pos.enable(TIME_STEP)
        self.rarmmotor_pos.enable(TIME_STEP)
        
        self.leftmotor.setPosition(float('inf'))
        self.rightmotor.setPosition(float('inf'))
        self.leftmotor.setVelocity(0.0)
        self.rightmotor.setVelocity(0.0)
        
        self.larmmotor.setPosition(0.)
        self.rarmmotor.setPosition(0.)
        self.leftmotor.setVelocity(MAX_SPEED)
        self.rightmotor.setVelocity(MAX_SPEED)
        self._setClawAngle(0.5)
        
        
        if self.name == 'red_robot':
            self.radio = Radio(channel=1)
            self.home = np.array([1., 1.])
        else:
            self.radio = Radio(channel=2)
            self.home = np.array([1., -1.])
        
        self.commands = {
            'SCN': self._scan,
            'MOV': self._move,
            'IDL': self._idle,
            'COL': self._collect,
            'IDN': self._identify,
            'RTN': self._return,
            'RLS': self._release,
        }
        
        # set the home position to return to with RTN statement
        
        
        
        # initialise these variables in __init__
        self.data = []
        self.command_time = 0 # keeps track of how long the current command has been run for
        
        self.cur_command = None
        self.cur_values = []        
        
        # display useful for debugging
        display = self.getDevice('display')
        self.display = MapDisplay(display)
        
    def clearQueue(self, **kwargs):
        """Finish any processing from the current command, then reset all variables"""
        self._idle()
        self.cur_command = None
        self.cur_values = []
        
        self.command_time = 0

        if self.data:
            # find the boxes, send data to shared controller
            box_coords = findClusters(self.data, self._getPos())
            for coord in box_coords:
                x, z = coord
                self.radio.send('BOX', x, z)
            np.save('listnp.npy', np.array(self.data)) 
            self.data = []
            
        if 'box_colour' in kwargs:
            self.radio.send('CLR', kwargs['box_colour'])
            
        
        x, z = self._getPos()
        self.radio.send('DNE', x, z)
        
             
    def runCommand(self):
        if self.cur_command is not None:
            self.cur_command(*self.cur_values)
    
    # IDL
    def _idle (self, *args):
        self._drive(0)
        if args:
            tot_time = args[0]
            if tot_time > 0 and self.command_time > tot_time:
                self.clearQueue()

    # SCN
    def _scan(self, *args):
        """Scan the environment with a full spin recording from the distance sensor"""
        
        # Testing red colour detection
        colour_values = self.camera.getImageArray()[0][0]
        as_colour = sum(c*16**(4-2*i) for i, c in enumerate(colour_values))
        self.display.drawPoint(np.array([0., 0.]), 20, as_colour) 
                
        #TODO make sure max radius is consistent i.e. from distance sensor or GPS centre
        R = .8
        tot_time = 5
        
         # completeness test
        if self.command_time > tot_time:
            self.clearQueue()
            return
        
        
        # Set spinning
        self._spin(4.3 / tot_time)
        
        # Record spatial information
        pos = self._getPos()
        d = self.dist_sensor.getValue() / 1000
        self.display.drawText(f'{d:.3f}', 10, 100)
        heading_vec = self._getBearing(as_vector=True)
        # Process dist sensor measurement

        
        # skip this point if distance is greater than the max radius
        if d > R: return
        
        # Find Target Coords
        pos_d = pos + DIST_TO_SENSOR * heading_vec
        pos_t = pos_d + d * heading_vec
                
                
        # skip this point if it lies on a wall
        if not pointInsideWalls(pos_t): return
        
        # Store for evaluation
        self.data.append(pos_t)
        
    # MOV
    def _move(self, *args, stop_on_obst=True):
        """Move to a point, with some basic collision avoidance along the way"""
        pos = self._getPos()
        target = np.array(args)
        v_targ = (target - pos) / np.linalg.norm(target - pos)
        v_head = self._getBearing(as_vector=True)
        theta = np.arccos(np.dot(v_targ, v_head))
        
        # compute cross product(v_targ, v_head) 
        # sign shows which direction to turn
        cross = v_head[0] * v_targ[1] - v_head[1] * v_targ[0]
        
        
        if theta < 0.1:
            dist_tolerance = 0.1
            speed = 1
            self._drive(1)
            
        else:
            dist_tolerance = 0.01
            
            # dot product determines spin speed
            # large dot product -> closely alligned -> small speed
            dot = np.clip(np.dot(v_head, v_targ), 0, 0.8)
            speed = np.sign(cross) * (1 - dot)
            self._spin(speed)
                
        # stop if:
        #  facing the right direction, and within 0.1 m
        #  not facing the right direction, and within 0.01m
        #  facing a block within 0.1m
        near_obst = (self.dist_sensor.getValue() / 1000) < 0.1 and stop_on_obst
        if near_obst or np.linalg.norm(target - pos) < dist_tolerance:
            self.clearQueue()
            return True
        
        
        self.display.drawPoint(pos, 3, 'red')
        
        self.display.drawLine(pos, pos + v_targ * 0.1, 'white', name='Target vector')
        self.display.drawPoint(target, 3, 'white')
        self.display.drawText('left' if cross<0 else 'right', 10, 450, 'white')
        
        self.display.drawLine(pos, pos + v_head * 0.1, 'blue', name='Heading vector')
        self.display.drawLegend()
        
        
    # COL
    def _collect(self, *args):
        """Once already facing a block, pick it up"""
        self._setClawAngle(-0.2)
        if self.command_time > 0.5: # small wait to avoid hitting the block
            self.clearQueue()
        
    # IDN
    def _identify(self, *args):
        """Line up with the box in front, then identify its colour and reverse"""
        red_channel = self.camera.getImageArray()[0][0][0]
        colour = 1. if red_channel > 50 else 0.
        # if it is the wrong colour, reverse for a bit
        if self.name == 'red_robot' and colour == 0.:
            self._drive(-0.5)
            if self.command_time > 1:
                self.clearQueue(box_colour=colour)
            
        else:
            self.clearQueue(box_colour=colour)
        
    # RTN
    def _return(self, *args):
        """Return home"""
        self._move(*self.home, stop_on_obst=False)
            
            
    # RLS
    def _release(self, *args):
        """Release currently held block"""
        self._setClawAngle(0.5)
        self._drive(-0.5)
        if self.command_time > 1:
            self.clearQueue()
        
    # Motor controls
    def _wheelMotors(self, v_left, v_right):
        self.leftmotor.setVelocity(v_left * MAX_SPEED)
        self.rightmotor.setVelocity(v_right * MAX_SPEED)
        
    def _setClawAngle(self, t):
        """Open (+ve) or close (-ve) claws"""
        self.larmmotor.setPosition(t)
        self.rarmmotor.setPosition(-t)
        
    def _drive(self, v):
        self._wheelMotors(v, v)
        
    def _spin(self, v):
        self._wheelMotors(v, -v)

    # Sensor reading
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
        self.step(TIME_STEP) # allow 1 time step to turn on sensors
        self.clearQueue()
        
        while self.step(TIME_STEP) != -1:
            self.display.clear()
            self.command_time += TIME_STEP / 1000

            
            # receive message
            msg = self.radio.receive()
            if msg is not None:
                print(self.name + ' received:\t', msg)
                cmd, *values = msg
                self.cur_command = self.commands.get(cmd, None)
                self.cur_values = values
                              
                
            # run the last command that was given
            self.runCommand()

   
robot = Collector()            
robot.run()