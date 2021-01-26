"""shared_controller controller."""

import struct
# https://docs.python.org/3/library/struct.html
import time

from controller import Robot, Emitter, Receiver

# create the Robot instance.
TIME_STEP = 32

def encodeCommand(cmd, v1, v2):
    """Encode a message using struct for emitting/receiving
    msg has form ('CMD', float, float)"""
    FORMAT = '3sdd' # 3-length string, float, float
    b = bytes(cmd, encoding='utf-8')
    msg = struct.pack(FORMAT, b, v1, v2)
    return msg


class Shared(Robot):
    def __init__(self):
        super().__init__()
              
        self.name = self.getName()
       
        
        # setup communication
        self.emitter = Emitter('emitter')
        self.emitter.setChannel(1)
        
        self.receiver = Receiver('receiver')
        self.receiver.enable(TIME_STEP)
        self.receiver.setChannel(1)
        
    def run(self):
        while robot.step(TIME_STEP) != -1:
            # send message to robot to move forward
            if time.perf_counter() % 1 < 0.1:
                msg = encodeCommand('SPN', 0., 0.)
                self.emitter.send(msg)
                
            
                            

robot = Shared()
robot.run()