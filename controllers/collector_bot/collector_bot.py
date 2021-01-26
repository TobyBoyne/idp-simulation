"""collector_bot controller."""

import struct

from controller import Robot, Emitter, Receiver

# create the Robot instance.

TIME_STEP = 32
MAX_SPEED = 6.28

# parameters based on the colour of the robot
ROBOT_PARAMS = {
    'red_bot': {'colour': 'red', 'channel': 1}
}


def decodeCommand(msg):
    FORMAT = '3sdd'
    b, v1, v2 = struct.unpack(FORMAT, msg)
    cmd = b.decode(encoding='utf-8')
    return cmd, v1, v2


class Collector(Robot):
    def __init__(self):
        super().__init__()
        
        # setup sensors
        self.gps = self.getDevice('gps')
        self.gps.enable(TIME_STEP)
        
        self.name = self.getName()
        
        # setup motors
        
        # setup communication
        self.emitter = Emitter('emitter')
        self.emitter.setChannel(1)
        
        self.receiver = Receiver('receiver')
        self.receiver.enable(TIME_STEP)
        self.receiver.setChannel(1)
        
        self.commands = {
            'SPN': self._spin,
            'MOV': self._move,
        }
        
        
    def runCommand(self, cmd, v1, v2):
        if cmd in self.commands:
            self.commands[cmd](v1, v2)
        
    def _spin(self, *args):
        """Spin one full rotation while reading from distance sensor"""
        print('Spinning')
        
    def _move(self, *args):
        """Move to a point, with some basic collision avoidance along the way"""
        print('Moving')
    
    def run(self):
        while robot.step(TIME_STEP) != -1:
            pos = self.gps.getValues()  

            # each loop, poll the receiver to see if there are any new commands from shared controller
            # if there are, decode the command and execute it
            if self.receiver.getQueueLength():
                packet = self.receiver.getData()
                self.receiver.nextPacket() # after being read, pop item from queue
                cmd, v1, v2 = decodeCommand(packet)
                print(cmd)
                
                self.runCommand(cmd, v1, v2)
            
            
            
robot = Collector()            
robot.run()