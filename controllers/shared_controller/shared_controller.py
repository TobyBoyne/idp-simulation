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
            'MRK': None, # MAKE
        }
        
        self.cmd_ids = {'red': 1, 'blue': 1}
        
        self.scan_route = [
            ['MOV', 0.6, 0.5],
            ['MOV', 0.1, 0.4],
            ['MOV', 0., 0.]
        ]
        
        self.data = []
        self.box_pos = np.full((8, 2), np.nan)
        self.boxes = {'red': [], 'blue': [], 'unknown': []}
        self.boxes_found = 0
        
    def _procedure(self, robot):
    
        """
        Main robot procedure
        Each number corresponds to a unique position in the flowchart
        The decision sequence updates the position and returns the associated command
        """
        
        cmd = self.cmd_ids[robot]
        
        
        if cmd in [1, 2, 3, 5]:
        
            if self.boxes_found < 8:
            
                if self.boxes[robot]:
                    
                    # Actually go to closest
                    pos = self.box_pos[self.boxes[robot].pop(0)]
                    self.cmd_ids[robot] = 7
                    return ('MOV', pos[0], pos[1])
                
                elif self.boxes['unknown']:
                    
                    pos = self.box_pos[self.boxes['unknown'].pop(0)]
                    self.cmd_ids[robot] = 5
                    return ('MOV', pos[0], pos[1])
                        
                else:
                    
                    if self.scan_route:
                        self.cmd_ids[robot] = 6
                        return self.scan_route.pop(0)
                    # More decisions here for route control
                    
            else:
                
                self.cmd_ids[robot] = 8
                return ('RTN', 0., 0.)
                
        elif cmd == 4:
            # self.boxes_found += 1
            self.cmd_ids[robot] = 3
            return ('RTN', 0., 0.)
            
        # elif cmd == 5:
        
            # self.cmd_ids[robot] = 2
            # return ('MRK', 0., 0.) 
            
        elif cmd == 6:
        
            self.cmd_ids[robot] = 1
            # update scan parameters
            return ('SCN', 1., 5.)
            
        elif cmd == 7:
        
            self.cmd_ids[robot] = 4
            # Collect
            return ('COL', 0., 0.)
        
                
    def _robotCmdDone(self, robot, *args):
        """Once a robot has completed a command, issue the next"""
        print('Robot has finished')
        
        if self.cmd_ids[robot] != 0:
            next_command = self._procedure(robot)
            print(next_command)
            self.red_radio.send(*next_command)
        else:
            path = os.path.join(os.getcwd(), '..', 'collector_bot', 'box_locations.npy')
            np.save(path, self.boxes)
            
                
    def _boxFound(self, robot, *args):
        """Process the location of a box discovered by a robot"""
        x, z = args
        new_box = np.array([x, z])
        
        # get all the boxes that have been found i.e. not NaNs
        boxes = self.box_pos[~np.isnan(self.box_pos[:, 0])]
        
        if boxes.shape[0] == 0:
            self.box_pos[0, :] = new_box
            self.boxes['unknown'].append(0)
            return
            
        dists = np.linalg.norm(boxes - new_box, axis=1)
            
        # if the box is near to another box, they are the same -> merge them
        if np.min(dists) < 0.05:
            closest_idx = np.argmin(dists)
            closest_box = boxes[closest_idx]
            boxes[closest_idx] = (closest_box + new_box) / 2
            return
            
        # if reach this point, recognise this as a new box
        empty_idxs = np.isnan(self.box_pos[:, 0])
        if not np.any(empty_idxs):
            print('Already found 8 boxes')
            return
            
        idx = np.argmax(empty_idxs)
        self.box_pos[idx, :] = new_box
        self.boxes['unknown'].append(idx)
        
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
        """Main loop"""
        
        next_command = self._procedure('red')
        print(next_command)
        self.red_radio.send(*next_command)
             
        while self.step(TIME_STEP) != -1:
            msg = self.red_radio.receive()
            if msg is not None:
                print(msg)
                cmd, *values = msg
                command = self.commands.get(cmd, None)
                if command is not None:
                    robot = 'red'
                    command(robot, *values)
        
                
             
shared = Shared()
shared.run()