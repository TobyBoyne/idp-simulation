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


def pointInsideWalls(p):
    x, y = p
    # L is slightly less than the actual half-width (1.2) to account for size of robot
    L = 1.1
    return -L <= x <= L and -L <= y <= L

def minDistance(u, v, B):
    """Find the minimum distance between the line segment defined by u and v, and all of the boxes B
    If v ~ u, then the two points are the same and no movement is needed"""
    # clamp the point to the line segment using parameter t
    # r = u + vt
    len_sqrd = np.linalg.norm(v) ** 2
    if len_sqrd < 1e-4: return np.inf
    t = np.clip(np.dot(B - u, v) / len_sqrd, 0, 1)
    projection = u + t[:, None] * v
    return np.min(np.linalg.norm(projection - B, axis=-1), initial=np.inf)


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
            'CLR': self._markBox,
        }
        
        self.cmd_ids = {'red': 1, 'blue': 1}
        
        self.data = []
        self.box_pos = np.full((8, 2), np.nan)
        self.boxes = {'red': [], 'blue': [], 'unknown': []}
        
        # create waypoint lists for each robot
        self.scan_route = {
            'red': self.plotScanRoute('red'),
            'blue': self.plotScanRoute('blue')
        }
        
        self.robot_pos = {'red': np.array([1., 1.]), 'blue': np.array([1., -1.])}
        
        # variable used to keep track of which box a robot has been sent to collect/mark
        # using idx, NOT position
        self.target_box = {'red': None, 'blue': None}
        
    def _procedure(self, robot):
    
        """
        Main robot procedure
        Each number corresponds to a unique position in the flowchart
        The decision sequence updates the position and returns the associated command
        """
        
        cmd = self.cmd_ids[robot]
        boxes_found = np.sum(self.allBoxesFound())
        print(self.boxes)
        
        if cmd == 3:
            # consider the box that is currently being held as collected
            box_idx = self.target_box[robot]
            self.boxes[robot].remove(box_idx)
            self.target_box[robot] = None
        
        if cmd in [1, 2, 3]:
        
            if boxes_found < 8:
                
                # TODO: improve target choice
                if self.boxes[robot]:
                    self.cmd_ids[robot] = 7

                    box_idx = self.boxes[robot][0]
                    self.target_box[robot] = box_idx
                    
                    box_x, box_z = self.box_pos[box_idx]
                    
                    return ('MOV', box_x, box_z)
                
                elif self.boxes['unknown']:
                    self.cmd_ids[robot] = 5
                    
                    box_idx = self.boxes['unknown'][0]
                    self.target_box[robot] = box_idx
                    
                    box_x, box_z = self.box_pos[box_idx]
                    
                    return ('MOV', box_x, box_z)
                        
                else:
                    # if there are no known boxes:
                    # move to the next point on the scanning route
                    # TODO: don't necessarily pop, as may need to return to that point
                    if self.scan_route[robot]:
                        self.cmd_ids[robot] = 6
                        waypoint = self.scan_route[robot].pop(0)
                        x, z = self.findPath(waypoint, self.robot_pos[robot])
                        return ('MOV', x, z)
                    
                    else:
                        # TODO: restart scanning route?
                        print('Scan route exhausted')
                        return ('RTN',)
                    
            else:
                
                self.cmd_ids[robot] = 8
                return ('RTN',)
                
        elif cmd == 4:
            self.cmd_ids[robot] = 3
            return ('RTN',)
            
        elif cmd == 5:

            self.cmd_ids[robot] = 2
            return ('IDN',) 
            
        elif cmd == 6:
        
            self.cmd_ids[robot] = 1
            # update scan parameters
            return ('SCN', 1., 5.)
            
        elif cmd == 7:
        
            self.cmd_ids[robot] = 4
            # Collect
            return ('COL',)
        
    # DNE           
    def _robotCmdDone(self, robot, *args):
        """Once a robot has completed a command, issue the next"""
        robot_x, robot_y = args
        self.robot_pos[robot] = np.array([robot_x, robot_y]) 
        
        
        if self.cmd_ids[robot] != 0:
            next_command = self._procedure(robot)
            if next_command is not None:
                self.red_radio.send(*next_command)
        else:
            path = os.path.join(os.getcwd(), '..', 'collector_bot', 'box_locations.npy')
            np.save(path, self.boxes)
    
    # CLR        
    def _markBox(self, robot, *args):
        """Assign the identified box to the correct colour"""
        box_idx = self.target_box[robot]
        box_colour = 'red' if args[0] else 'blue'
        
        if box_idx in self.boxes['unknown']:
            self.boxes['unknown'].remove(box_idx)
            self.boxes[box_colour].append(box_idx)
            
        else:
            print('Marked box was not unknown')
            
                
    def _boxFound(self, robot, *args):
        """Process the location of a box discovered by a robot"""
        x, z = args
        new_box = np.array([x, z])
        
        
        # get all the boxes that have been found i.e. not NaNs
        # boxes = self.box_pos[~np.isnan(self.box_pos[:, 0])]
        all_boxes = self.allBoxesFound()
        boxes = self.box_pos[all_boxes]
        
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
        # empty_idxs = np.isnan(self.box_pos[:, 0])
        empty_idxs = ~ all_boxes
        if not np.any(empty_idxs):
            print('Already found 8 boxes')
            return
            
        idx = np.argmax(empty_idxs)
        self.box_pos[idx, :] = new_box
        self.boxes['unknown'].append(idx)
        
        
    def allBoxesFound(self):
        """Return a boolean array of the boxes currently on the field
        Once a box has been collected, no longer picked up by this function"""
        all_boxes = self.boxes['red'] + self.boxes['blue'] + self.boxes['unknown']
        L = self.box_pos.shape[0]
        return np.isin(np.arange(L), all_boxes)
        
    def plotScanRoute(self, robot, R=0.4, step_multiple=1.):
        """Create a series of points that make the robot take a path to scan the field"""
        start = np.array([0.92, 0.95])
        step = -step_multiple * R
        
        X = np.arange(start[0], -1.1, step)
        Z = np.arange(start[1], 0, step)
        
        # plot a 'snaking' path across half of arena
        path_xs = np.concatenate([X[::(-1)**(i%2)] for i in range(len(Z))])
        path_zs = np.repeat(Z, len(X))
        
        # flip in x-y plane for blue robot
        if robot == 'blue':
            path_zs = - path_zs
        
        
        # create a list of commands to move and scan at each point
        waypoints = []
        for x, z in zip(path_xs, path_zs):
            waypoints.append(np.array([x, z]))
            
        return waypoints
        
        
    def findPath(self, pt, pos):
        """Find the most viable path to take to get to a waypoint"""
        CLEARANCE = 0.2 # min distance between robot centre and box
        move_vec = pt - pos
        found_boxes = self.box_pos[self.allBoxesFound()]
        # rotate the movement vector until a possible direction to move in is found
        for i in range(100):
            t = i * (np.pi / 50) * (-1)**(i % 2)
            # TODO: check target is within arena!
            if pointInsideWalls(pos + move_vec):
                if minDistance(pos, move_vec, found_boxes) > CLEARANCE:
                    # return this as the target vector
                    return pos + move_vec
    
            rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            move_vec = np.dot(rot, pt - pos)
        

        raise RuntimeError('Stuck in a loop; cannot find a clear path')
                
    def run(self):
        """Main loop"""
        
        next_command = self._procedure('red')
        print(next_command)
        self.red_radio.send(*next_command)
             
        while self.step(TIME_STEP) != -1:
            msg = self.red_radio.receive()
            if msg is not None:
                print('Shared received:\t', msg)
                cmd, *values = msg
                command = self.commands.get(cmd, None)
                if command is not None:
                    robot = 'red'
                    command(robot, *values)
        
                
             
shared = Shared()
shared.run()