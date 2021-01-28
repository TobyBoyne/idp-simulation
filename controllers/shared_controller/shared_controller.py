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
        # only send the spin message once
        msg = ('SPN', 0., 0.)
        self.red_radio.send(*msg)
        
        while robot.step(TIME_STEP) != -1:           
            packet = self.red_radio.receive()
            if time.perf_counter() > 40:
                self.red_radio.send('STP', 0., 0.)
            
            
            if packet is not None:
                
                print(packet)
                pass
                
             
robot = Shared()
robot.run()