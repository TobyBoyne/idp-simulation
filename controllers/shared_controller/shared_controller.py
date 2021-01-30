"""shared_controller controller."""

import time
import numpy as np

# import modules in parent directory 
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from radio import Radio

from controller import Robot

TIME_STEP = 32

class Shared(Robot):
    def __init__(self):
        super().__init__()
             
        # setup communication
        self.red_radio = Radio(channel=1,
            emitter_name='emitter_red', receiver_name='receiver_red')
        self.blue_radio = Radio(channel=2,
            emitter_name='emitter_blue', receiver_name='receiver_blue')
        
        self.data = []
        
        
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
            commands.append(['SPN', 0., 0.])
            
        return commands
        
    def run(self):
        
        # Send spin message and wait for reply
        # msg = ('SCN', 0., 0.)
        msg = ('MOV', 0.2, 0.2)
        self.red_radio.send(*msg)
        self.plotScanRoute()
        
        
        while robot.step(TIME_STEP) != -1:
            pass
           
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
        
                
             
robot = Shared()
robot.run()