import numpy as np
import smbus

class vehicleClass:
    
    def __init__(self):
        self.modeAutonomous = False
        self.x_axis = 0
        self.y_axis = 0
        self.servo1_pos = 0
        self.servo2_pos = 0
        self.detector1_state = False
        self.detector2_state = False
        self.bus = smbus.SMBus(1)
        self.adress=0x08

    def __str__(self):
        return("AutonomousMode: {} \n Coordinates: (X:{} Y:{}) \n Servos:(1:{} | 2: {}) \n Detectors:(1:{} | 2: {}) ".format(self.modeAutonomous,self.x_axis,self.y_axis,self.servo1_pos,self.servo2_pos,self.detector1_state,self.detector2_state))

    def sendToSlave(self):
        data = [0 for t in range(5)]
        data[0] = lambda x: 1 if self.modeAutonomous else 0
        data[1] = self.calculateLeftTrack()
        data[2] = self.calculateRightTrack()
        data[3] = self.servo1_pos + 10
        data[4] = self.servo2_pos + 10

        self.bus.write_block_data(self.adress,0,data)



    def calculateLeftTrack(self):
        val = -self.y_axis + self.x_axis + 100
        val = np.clip(val, 0, 200)
        return val

    def calculateRightTrack(self):
        val = -self.y_axis - self.x_axis + 100
        val = np.clip(val, 0, 200)
        return val
