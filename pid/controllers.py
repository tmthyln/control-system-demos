from time import time


class PIDController:
    
    def __init__(self, p: float = 0, i: float = 0, d: float = 0):
        self.p = p
        
        self.i = i
        self.accum = 0
        
        self.d = d
        self.last_measurement = None
        self.last_time_sec = time()
    
    def reset(self):
        self.accum = 0
    
    def compute(self, measurement, setpoint):
        error = setpoint - measurement
        self.accum += error
        
        if self.last_measurement is None:
            diff = 0
        else:
            diff = (measurement - self.last_measurement) / (time() - self.last_time_sec)
        self.last_time_sec = time()
        self.last_measurement = measurement
        
        return self.p * error + self.i * self.accum + self.d * diff
