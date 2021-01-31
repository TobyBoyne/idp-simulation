"""Radio class used for two-way communication between the robot and shared controller"""

import struct
# https://docs.python.org/3/library/struct.html

from controller import Emitter, Receiver

TIME_STEP = 16

# format to encode the message using struct
#   3s:string of length 3 = command
#   2d:2 floats (double)  = arguments
FORMAT = '3s2d'

# encoding used for turning string to bytes
ENCODING = 'utf-8'


def encodeCommand(cmd, v1=0., v2=0.):
    b = bytes(cmd, encoding=ENCODING)
    msg = struct.pack(FORMAT, b, v1, v2)
    return msg
    
def decodeCommand(msg):
    b, v1, v2 = struct.unpack(FORMAT, msg)
    cmd = b.decode(encoding=ENCODING)
    return cmd, v1, v2
    

class Radio():
    def __init__(self, channel, emitter_name='emitter', receiver_name='receiver'):
        self.emitter = Emitter(emitter_name)
        self.emitter.setChannel(channel)
        
        self.receiver = Receiver(receiver_name)
        self.receiver.enable(TIME_STEP)
        self.receiver.setChannel(channel)
        
    def send(self, *args):
        """Takes arbitrary number of arguments for encoding"""
        msg = encodeCommand(*args)
        self.emitter.send(msg)
        
    def receive(self, log=False):
        """Poll receiving queue, to check if messages have been sent
        Return None if there are no messages"""
        if self.receiver.getQueueLength():
            packet = self.receiver.getData()
            self.receiver.nextPacket() # after being read, pop item from queue
            return decodeCommand(packet)
        else:
            return None