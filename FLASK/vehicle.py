class vehicleClass:
    
    def __init__(self):
        self.modeAutonomous = False
        self.x_axis = 0
        self.y_axis = 0
        self.servo1_pos = 0
        self.servo2_pos = 0
        self.detector1_state = False
        self.detector2_state = False

    def __str__(self):
        return("AutonomousMode: {} \n Coordinates: (X:{} Y:{}) \n Servos:(1:{} | 2: {}) \n Detectors:(1:{} | 2: {}) ".format(self.modeAutonomous,self.x_axis,self.y_axis,self.servo1_pos,self.servo2_pos,self.detector1_state,self.detector2_state))