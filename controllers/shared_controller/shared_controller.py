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

# TODO: on return, pick avoid blocks

def targetInsideWalls(robot, p, on_side=None):
    x, z = p
    # L is slightly less than the actual half-width (1.2) to account for size of robot
    # only allow movement within current half of the arena
    L = 1.1
    top_right = np.array([L, L * (on_side != -1)])
    bottom_left = np.array([-L, -L * (on_side != 1)])
    return np.logical_and(p <= top_right, p >= bottom_left).all()
    

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

SCAN_R = 0.8
SCAN_STEP = 0.8
def plotScanRoute(robot):
    """Create a series of points that make the robot take a path to scan the field"""
    R = SCAN_R
    step_multiple = SCAN_STEP
    
    start = np.array([.8, .8])
    step = -step_multiple * R

    X = np.arange(start[0], -1.1, step)
    Z = np.arange(start[1], 0, step)

    # plot a 'snaking' path across half of arena
    path_xs = np.concatenate([X[::(-1) ** (i % 2)] for i in range(len(Z))])
    path_zs = np.repeat(Z, len(X))

    # flip in x-y plane for blue robot
    if robot == 'blue':
        # path_zs = - path_zs
        path_zs = path_zs - 1.1

    # create a list of commands to move and scan at each point
    waypoints = []
    for x, z in zip(path_xs, path_zs):
        waypoints.append(np.array([x, z]))
    print(f'{len(waypoints)} waypoints per robot')
    return waypoints


class DummyRobot:
    """Class used to store information about the robot
    All information is local - not pulled directly from CollectorBot"""
    def __init__(self, colour):
        self.colour = colour
        # position only updated at each DNE command
        if colour == 'red':
            self.home = np.array([1., 1.])
            channel = 1
            self.arena_side = 1 # which side of the x axis
        else:
            self.home = np.array([1., -1.])
            channel = 2
            self.arena_side = -1
        
        self.pos = self.home
        
        self.radio = Radio(channel=channel,
            emitter_name=f'emitter_{colour}', receiver_name=f'receiver_{colour}')

        self.cmd_id = 1 # current position on flow chart
        self.scan_route = plotScanRoute(colour) # list of waypoints to scan
        self.target_box = None # box currently being collected
        
        # keeping track of overall status
        self.scan_complete = False
        self.complete = False

    def __eq__(self, other):
        if isinstance(other, str):
            return self.colour == other
        elif isinstance(other, DummyRobot):
            return self.colour == other.colour


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
        
        display = self.getDevice('sharedDisplay')
        self.display = MapDisplay(display)
        
        self.first_to_finish = ''

    def _procedure(self, robot):
    
        """
        Main robot procedure
        Each number corresponds to a unique position in the flowchart
        The decision sequence updates the position and returns the associated command
        """
        
        cmd = robot.cmd_id
        # print(f'{robot.colour}: {cmd}')
        
        if cmd in [1, 2, 9]:
                
            box_lists = (self.boxes[robot.colour], self.boxes['unknown'])
            for box_list, next_cmd in zip(box_lists, (7, 5)):
                # check that the box_list is not empty
                if not box_list: continue
                box_idxs = np.array(box_list)
                box_pos = self.box_pos[box_list]
                # filter to only include those on the same side
                if robot.arena_side is not None:
                    same_side = box_pos[:, 1] * robot.arena_side > 0
                    box_pos = box_pos[same_side]
                    box_idxs = box_idxs[same_side]
                
                # filter to not include the box that is targetted by the other robot
                already_targeted = box_idxs == self.otherBot(robot).target_box
                box_pos = box_pos[~already_targeted]
                box_idxs = box_idxs[~already_targeted]
                if box_pos.shape[0] == 0: continue

                # TODO: find nearest box
                nearest = np.argmin(np.linalg.norm(box_pos, axis=-1))
                idx = box_idxs[nearest]                
                
                robot.target_box = idx
                
                (x, z), full_length = self.findPath(robot, self.box_pos[idx], 
                    ignore_self_colour=True, on_side=robot.arena_side)
                
                if full_length:
                    # only collect if the box has been reached
                    robot.cmd_id = next_cmd
                
                
                return ('MOV', x, z)
                    
            else: # nobreak
                # if there are no known boxes:
                # move to the next point on the scanning route
                # TODO: don't necessarily pop, as may need to return to that point
                if robot.scan_route:
                    waypoint = robot.scan_route[0]
                    (x, z), full_length = self.findPath(robot, waypoint, 
                        on_side=robot.arena_side)
                    # if the robot does not move the full length, move again
                    if full_length:
                        robot.scan_route.pop(0)
                    robot.cmd_id = 6
                    
                    return ('MOV', x, z)
                
                else:
                    if not robot.scan_complete:
                        print(f'{robot.colour} scan route exhausted')
                        robot.scan_complete = True
                    remaining_boxes = self.boxes[robot.colour] + self.boxes['unknown']
                    
                    if remaining_boxes:
                    
                        # wait until the other robot has completely finished
                        other_bot = self.otherBot(robot)
                        if not self.first_to_finish and not other_bot.scan_complete:
                            self.first_to_finish = robot.colour

                        
                        if robot.colour == self.first_to_finish:
                            if not other_bot.complete:
                                if robot.complete:
                                    robot.target_box = None
                                    return ('IDL', 1.)
                                robot.complete = True
                                return ('RTN', *robot.home)
                                
                    
                        robot.cmd_id = 3
                        robot.target_box = None
                        robot.arena_side = None
                        return ('RTN', *robot.home)
                    else:
                        # all boxes found - complete!
                        robot.complete = True
                        robot.cmd_id = 8
                        robot.target_box = None
                        return ('RTN', *robot.home)
                        

                
        elif cmd == 3:
            print(self.boxes)
    
            robot.cmd_id = 9
            # release, remove from self.boxes
            box_idx = robot.target_box
            if box_idx is not None:
                self.boxes[robot.colour].remove(box_idx)
                robot.target_box = None
            return ('RLS',)
                
        elif cmd == 4:
            # robot.cmd_id = 3
            # TODO: return home with collision avoidance
            (x, z), full_length = self.findPath(robot, robot.home, 
                ignore_self_colour=True, MAX_DISTANCE=0.3)
            if full_length:
                robot.cmd_id = 3
            return ('RTN', x, z)
            
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
        box_found = args[1]
        
        if box_idx in self.boxes['unknown']:
            self.boxes['unknown'].remove(box_idx)
            
            # only append the colour if the box was found (i.e. not missing)
            if box_found:
                self.boxes[box_colour].append(box_idx)
            
        else:
            print('Marked box was not unknown')
            
    # BOX            
    def _boxFound(self, robot, *args):
        """Process the location of a box discovered by a robot"""
        x, z = args
        new_box = np.array([x, z])
        
        # if the box is on the other side of the map, assume that it is the other robot
        if robot.arena_side is not None and z * robot.arena_side < 0:
            return
        
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
        
        
    def otherBot(self, robot):
        return self.robots['red'] if robot=='blue' else self.robots['blue']    
    
    def allBoxesFound(self, avoid_colour=None):
        """Return a boolean array of the boxes currently on the field
        Once a box has been collected, no longer picked up by this function"""
        if avoid_colour is None:
            all_boxes = self.boxes['red'] + self.boxes['blue'] + self.boxes['unknown']
        else:
            if avoid_colour == 'blue': all_boxes = self.boxes['blue']
            if avoid_colour == 'red':  all_boxes = self.boxes['red']
        L = self.box_pos.shape[0]
        return np.isin(np.arange(L), all_boxes)
        

        
    def findPath(self, robot, pt, ignore_self_colour=False, MAX_DISTANCE=SCAN_R, on_side=None):
        """Find the most viable path to take to get to a waypoint"""
        # return True if was able to move the full distance
        # ignore_self_colour needed for collecting blocks
        
        CLEARANCE = 0.2 # min distance between robot centre and box
        
        pos = robot.pos
        base_move_vec = pt - pos
        
        full_length = True
        move_length = np.linalg.norm(base_move_vec)
        if move_length > MAX_DISTANCE:
            base_move_vec = base_move_vec * (MAX_DISTANCE / move_length)
            full_length = False
        
        move_vec = base_move_vec
        self.display.drawLine(pos, pos + move_vec)
        print(pos + move_vec)
        
        if ignore_self_colour:
            other_colour = self.otherBot(robot).colour
            found_boxes = self.box_pos[self.allBoxesFound(avoid_colour=other_colour)]
        else:
            found_boxes = self.box_pos[self.allBoxesFound()]
        # rotate the movement vector until a possible direction to move in is found
        for i in range(100):
            t = i * (np.pi / 50) * (-1)**(i % 2)
            if targetInsideWalls(robot, pos + move_vec, on_side):
                # filter out all blocks that are outside FOV
                # only include where dot > 0 i.e. in front of box
                box_vecs = found_boxes - pos
                dots = np.dot(box_vecs, move_vec)
                boxes_in_fov = found_boxes[dots > 0]            
                if minDistance(pos, move_vec, boxes_in_fov) > CLEARANCE:
                    return pos + move_vec, full_length
    
            rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
            move_vec = np.dot(rot, base_move_vec)
        

        print('Cannot find a clear path')
        # if a clear path cannot be found, reverse
        move_vec = pos - pt
        move_vec = move_vec * (0.1 / np.linalg.norm(move_vec))
        return pos + move_vec, False
                                
    def run(self):
        """Main loop"""
        for robot in self.robots.values():
            next_command = self._procedure(robot)
            robot.radio.send(*next_command)
             
        while self.step(TIME_STEP) != -1:
            for robot in self.robots.values():
                msg = robot.radio.receive()
                if msg is not None:
                    if msg[0] != 'DNE':
                        print(f'Shared (from {robot.colour}):\t {msg}')
                    cmd, *values = msg
                    command = self.commands.get(cmd, None)
                    if command is not None:
                        command(robot, *values)
                        
                self.display.clear()
                for colour, idxs in self.boxes.items():
                    if colour == 'unknown': colour = 'white'
                    for idx in idxs:
                        self.display.drawPoint(self.box_pos[idx], 5, colour)
        
                
             
shared = Shared()
shared.run()