"""shared_controller controller."""

import time
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio

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
        
    def _robotCmdDone(self, robot, *args):
        """Once a robot has completed a command, issue the next"""
        print('Robot has finished')
        if len(self.red_queue) > 0:
            next_command = self.red_queue.pop(0)
            self.red_radio.send(*next_command)
        
        
    def _boxFound(self, robot, *args):
        """Process the location of a box discovered by a robot"""
        pass
        
        
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
            commands.append(['SCN', 0., 0.])
            
        return commands
        
    def run(self):
        
        # Send spin message and wait for reply
        # msg = ('SCN', 0., 0.)
        
        # move to start
        msg = ('MOV', 1., 1.)
        self.red_radio.send(*msg)
        # self.red_queue = self.plotScanRoute()
        self.red_queue = [['SCN', 0.4, 35]]
        
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