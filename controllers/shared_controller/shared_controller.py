"""shared_controller controller."""

import time

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
        
        
    def run(self):
        
        # Send spin message and wait for reply
        msg = ('SCN', 0., 0.)
        self.red_radio.send(*msg)
        
        
        # Can we dispense with inheriting from robot?
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