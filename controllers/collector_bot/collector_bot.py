"""collector_bot controller."""

import time
import math
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio
from display import MapDisplay

from controller import Robot, DistanceSensor, Camera


TIME_STEP = 16
MAX_SPEED = 6.28

DIST_TO_SENSOR = 0.12

# parameters based on the colour of the robot
ROBOT_PARAMS = {
    'red_bot': {'colour': 'red', 'channel': 1}
}

        
def pointInsideWalls(p):
    x, y = p
    # L is slightly less than the actual width to account for noise in distance sensor
    L = 1.16
    return -L <= x <= L and -L <= y <= L

def findClusters(data_lst, pos):
    """Find the clusters within the scanned datapoints"""
    data = np.array(data_lst)
    centroids = data[0, None]
    min_R = 0.1

    # if there are few datapoints, break out early
    if data.shape[0] < 5:
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
        
        self.dist_sensor = self.getDevice('distance1')
        self.dist_sensor.enable(TIME_STEP)
        
        self.camera = self.getDevice('camera')
        self.camera.enable(TIME_STEP)
        
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
        self.scan_time = 0
        
        self.clearQueue()
        
        # display useful for debugging
        display = self.getDevice('display')
        self.display = MapDisplay(display)
        
    def clearQueue(self):
        self._drive(0)
        self.cur_command = None
        self.cur_values = []
        
        self.scan_time = 0
        
        if self.data:
            # find the boxes, send data to shared controller
            box_coords = findClusters(self.data, self._getPos())
            for coord in box_coords:
                x, z = coord
                self.radio.send('BOX', x, z)
            np.save('listnp.npy', np.array(self.data)) 
            self.data = []
        
             
    def runCommand(self):
        if self.cur_command is not None:
            self.cur_command(*self.cur_values)


    def _scan(self, *args):
        """Scan the environment with a full spin recording from the distance sensor"""
        # first arg is the radius of the scan
        # second arg is total time to scan for
        
        # Testing red colour detection
        print(str(self.camera.getImageArray()[0][0][0]))
        
        R = args[0]
        tot_time = args[1]
        
         # completeness test
        self.scan_time += TIME_STEP / 1000
        if self.scan_time > tot_time:
            self.clearQueue()
            self.radio.send('DNE')
            return
        
        
        # Set spinning
        self._spin(0.10)
        
        # Record spatial information
        pos = self._getPos()
        d = self.dist_sensor.getValue()
        heading_vec = self._getBearing(as_vector=True)
        # Process dist sensor measurement
        d = 0.7611 * (d**(-0.9313)) - 0.1252
        
        # skip this point if distance is greater than the max radius
        if d > R: return
        
        # Find Target Coords
        pos_d = pos + DIST_TO_SENSOR * heading_vec
        pos_t = pos_d + d * heading_vec
                
                
        # skip this point if it lies on a wall
        if not pointInsideWalls(pos_t): return
        
        # Store for evaluation
        self.data.append(pos_t)
        

    def _move(self, *args):
        """Move to a point, with some basic collision avoidance along the way"""
        
        pos = self._getPos()
        target = np.array(args)
        v_targ = (target - pos) / np.linalg.norm(target - pos)
        v_head = self._getBearing(as_vector=True)
        theta = np.arccos(np.dot(v_targ, v_head))
        
        # compute cross product(v_targ, v_head) 
        # sign shows which direction to turn
        cross = v_head[0] * v_targ[1] - v_head[1] * v_targ[0]
        
        # dot product determines spin speed
        # large dot product -> closely alligned -> small speed
        dot = np.clip(np.dot(v_head, v_targ), 0, 0.8)
        
        if theta < 0.1:
            
            speed = np.array([1, 1])
            
            if np.linalg.norm(target - pos) < 0.1:
                
                speed = np.array([0, 0])
            
                self.clearQueue()
                self.radio.send('DNE', 0., 0.)
            
        else:
            
            speed = np.sign(cross) * (1 - dot) * np.array([1, -1])
        
        self._wheelMotors(speed[0], speed[1])
        
        
        self.display.drawPoint(pos, 3, 'red')
        
        self.display.drawLine(pos, pos + v_targ * 0.1, 'white', name='Target vector')
        self.display.drawPoint(target, 3, 'white')
        self.display.drawText('left' if cross<0 else 'right', 10, 450, 'white')
        
        self.display.drawLine(pos, pos + v_head * 0.1, 'blue', name='Heading vector')
        self.display.drawLegend()
                
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
            self.display.clear()

            
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