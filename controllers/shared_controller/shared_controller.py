"""shared_controller controller."""

import time
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio
from display import MapDisplay

from controller import Robot

TIME_STEP = 16

class Shared(Robot):
    def __init__(self):
        super().__init__()
             
        # setup communication
        self.red_radio = Radio(channel=1,
            emitter_name='emitter_red', receiver_name='receiver_red')
        self.blue_radio = Radio(channel=2,
            emitter_name='emitter_blue', receiver_name='receiver_blue')
        
        self.commands = {
            'DNE': self._robotCmdDone,
            'BOX': self._boxFound,
        }
        
        self.red_queue = []
        self.blue_queue = []
        
        self.data = []
        self.boxes = np.full((8, 2), np.nan)
        self.boxes_found = {'red': [], 'blue': [], 'unknown': []}
                
    def _robotCmdDone(self, robot, *args):
        """Once a robot has completed a command, issue the next"""
        print('Robot has finished')
        if len(self.red_queue) > 0:
            next_command = self.red_queue.pop(0)
            self.red_radio.send(*next_command)
        else:
            path = os.path.join(os.getcwd(), '..', 'collector_bot', 'box_locations.npy')
            np.save(path, self.boxes)
                
    def _boxFound(self, robot, *args):
        """Process the location of a box discovered by a robot"""
        x, z = args
        new_box = np.array([x, z])
        
        # get all the boxes that have been found i.e. not NaNs
        boxes = self.boxes[~np.isnan(self.boxes[:, 0])]
        
        if boxes.shape[0] == 0:
            self.boxes[0, :] = new_box
            return
            
        dists = np.linalg.norm(boxes - new_box, axis=1)
            
        # if the box is near to another box, they are the same -> merge them
        if np.min(dists) < 0.05:
            closest_idx = np.argmin(dists)
            closest_box = boxes[closest_idx]
            boxes[closest_idx] = (closest_box + new_box) / 2
            return
            
        # if reach this point, recognise this as a new box
        empty_idxs = np.isnan(self.boxes[:, 0])
        if not np.any(empty_idxs):
            print('Already found 8 boxes')
            return
            
        idx = np.argmax(empty_idxs)
        self.boxes[idx, :] = new_box
        
    def plotScanRoute(self, R=0.4, step_multiple=1.):
        """Create a series of commands that make the robot take a path to scan the field"""
        start = np.array([0.92, 0.95])
        step = -step_multiple * R
        
        X = np.arange(start[0], -1.1, step)
        Z = np.arange(start[1], 0, step)
        
        # plot a 'snaking' path across half of arena
        path_xs = np.concatenate([X[::(-1)**(i%2)] for i in range(len(Z))])
        path_zs = np.repeat(Z, len(X))
        
        # create a list of commands to move and scan at each point
        commands = []
        for x, z in zip(path_xs, path_zs):
            commands.append(['MOV', x, z])
            commands.append(['SCN', R, 35])
            
        return commands
        
    def run(self):
        
        # Send spin message and wait for reply
        # msg = ('SCN', 0., 0.)
        
        # move to start
        msg = ('MOV', 1., 1.)
        self.red_radio.send(*msg)
        # self.red_queue = self.plotScanRoute()
        self.red_queue = [
            ['SCN', 0.4, 20],
            ['MOV', 0.6, 0.5],
            ['SCN', 0.4, 20],
            ['MOV', 0.1, 0.4],
            ['SCN', 0.4, 20],
            ['MOV', 0., 0.],
            ['SCN', 0.4, 20],
        ]
        
        while self.step(TIME_STEP) != -1:
            msg = self.red_radio.receive()
            if msg is not None:
                print(msg)
                cmd, *values = msg
                command = self.commands.get(cmd, None)
                if command is not None:
                    robot = 'red'
                    command(robot, *values)
           
        """
        packet = None
        while packet == None:
            packet = self.red_radio.receive()
            # timeout escape
        packet = None
        
        
        # process data
        np.save('listnp.npy', np.array(self.data)) 
        """
        
        # Next instruction
        self.red_radio.send('MOV', 0., 0.)
        
                
             
shared = Shared()
shared.run()