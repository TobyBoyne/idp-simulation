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

def plotScanRoute(robot, R=0.8, step_multiple=1.):
    """Create a series of points that make the robot take a path to scan the field"""
    start = np.array([1., 1.])
    step = -step_multiple * R

    X = np.arange(start[0], -1.1, step)
    Z = np.arange(start[1], 0, step)

    # plot a 'snaking' path across half of arena
    path_xs = np.concatenate([X[::(-1) ** (i % 2)] for i in range(len(Z))])
    path_zs = np.repeat(Z, len(X))

    # flip in x-y plane for blue robot
    if robot == 'blue':
        path_zs = - path_zs

    # create a list of commands to move and scan at each point
    waypoints = []
    for x, z in zip(path_xs, path_zs):
        waypoints.append(np.array([x, z]))

    return waypoints


class DummyRobot:
    """Class used to store information about the robot
    All information is local - not pulled directly from CollectorBot"""
    def __init__(self, colour):
        self.colour = colour
        # position only updated at each DNE command
        self.pos = np.array([1., 1.]) if colour=='red' else np.array([1., -1.])
        
        self.radio = Radio(channel=1 if colour=='red' else 2,
            emitter_name=f'emitter_{colour}', receiver_name=f'receiver_{colour}')

        self.cmd_id = 1 # current position on flow chart
        self.scan_route = plotScanRoute(colour) # list of waypoints to scan
        self.target_box = None # box currently being collected

    def __eq__(self, other):
        if isinstance(other, str):
            return self.colour == other


class Shared(Robot):
    def __init__(self):
        super().__init__()

        self.robots = {'red': DummyRobot('red'), 'blue': DummyRobot('blue')}

        self.commands = {
            'DNE': self._robotCmdDone,
            'BOX': self._boxFound,
            'CLR': self._markBox,
        }
        

        # keep track of box positions, and use index in box_pos to identify colours
        self.box_pos = np.full((8, 2), np.nan)
        self.boxes = {'red': [], 'blue': [], 'unknown': []}

    def _procedure(self, robot):
    
        """
        Main robot procedure
        Each number corresponds to a unique position in the flowchart
        The decision sequence updates the position and returns the associated command
        """
        
        cmd = robot.cmd_id
        boxes_found = np.sum(self.allBoxesFound())

        # TODO - turn into release
        # if cmd == 3:
            # consider the box that is currently being held as collected
            # print(self.boxes)
            # box_idx = robot.target_box
            # self.boxes[robot.colour].remove(box_idx)
            # robot.target_box = None
        
        if cmd in [1, 2, 9]:
            if cmd == 9:
                # release, remove from self.boxes
                box_idx = robot.target_box
                self.boxes[robot.colour].remove(box_idx)
                robot.target_box = None
                
        
            if boxes_found < 8:
                
                # TODO: improve target choice
                if self.boxes[robot.colour]:
                    robot.cmd_id = 7

                    box_idx = self.boxes[robot.colour][0]
                    robot.target_box = box_idx
                    
                    box_x, box_z = self.box_pos[box_idx]
                    
                    return ('MOV', box_x, box_z)
                
                elif self.boxes['unknown']:
                    robot.cmd_id = 5
                    
                    box_idx = self.boxes['unknown'][0]
                    robot.target_box = box_idx
                    
                    box_x, box_z = self.box_pos[box_idx]
                    
                    return ('MOV', box_x, box_z)
                        
                else:
                    # if there are no known boxes:
                    # move to the next point on the scanning route
                    # TODO: don't necessarily pop, as may need to return to that point
                    if robot.scan_route:
                        robot.cmd_id = 6
                        waypoint = robot.scan_route.pop(0)
                        x, z = self.findPath(waypoint, robot.pos)
                        return ('MOV', x, z)
                    
                    else:
                        # TODO: restart scanning route?
                        print('Scan route exhausted')
                        robot.cmd_id = 8
                        return ('RTN',)
                    
            else:
                
                robot.cmd_id = 8
                return ('RTN',)
                
        elif cmd == 3:
            robot.cmd_id = 9
            return ('RLS',)
                
        elif cmd == 4:
            robot.cmd_id = 3
            return ('RTN',)
            
        elif cmd == 5:

            robot.cmd_id = 2
            return ('IDN',) 
            
        elif cmd == 6:
        
            robot.cmd_id = 1
            return ('SCN',)
            
        elif cmd == 7:
        
            robot.cmd_id = 4
            # Collect
            return ('COL',)
        
        
    # DNE           
    def _robotCmdDone(self, robot, *args):
        """Once a robot has completed a command, issue the next"""
        robot_x, robot_y = args
        robot.pos = np.array([robot_x, robot_y])
        
        
        if robot.cmd_id != 0:
            next_command = self._procedure(robot)
            if next_command is not None:
                robot.radio.send(*next_command)
        else:
            path = os.path.join(os.getcwd(), '..', 'collector_bot', 'box_locations.npy')
            np.save(path, self.boxes)
    
    # CLR        
    def _markBox(self, robot, *args):
        """Assign the identified box to the correct colour"""
        box_idx = robot.target_box
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
        

        
    def findPath(self, pt, pos):
        """Find the most viable path to take to get to a waypoint"""
        CLEARANCE = 0.2 # min distance between robot centre and box
        move_vec = pt - pos
        found_boxes = self.box_pos[self.allBoxesFound()]
        # rotate the movement vector until a possible direction to move in is found
        for i in range(100):
            t = i * (np.pi / 50) * (-1)**(i % 2)
            if pointInsideWalls(pos + move_vec):
                # filter out all blocks that are outside FOV
                # only include where dot > 0 i.e. in front of box
                box_vecs = found_boxes - pos
                dots = np.dot(box_vecs, move_vec)
                boxes_in_fov = found_boxes[dots > 0]            
                if minDistance(pos, move_vec, boxes_in_fov) > CLEARANCE:
                    # return this as the target waypoint
                    #TODO return command, cmd==RVS if stuck
                    return pos + move_vec
    
            rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            move_vec = np.dot(rot, pt - pos)
        

        raise RuntimeError('Stuck in a loop; cannot find a clear path')
                
    def run(self):
        """Main loop"""
        red_robot = self.robots['red']
        next_command = self._procedure(red_robot)
        print(next_command)
        red_robot.radio.send(*next_command)
             
        while self.step(TIME_STEP) != -1:
            for robot in self.robots.values():
                msg = robot.radio.receive()
                if msg is not None:
                    print('Shared received:\t', msg)
                    cmd, *values = msg
                    command = self.commands.get(cmd, None)
                    if command is not None:
                        command(robot, *values)
        
                
             
shared = Shared()
shared.run()