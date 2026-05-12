import numpy as np

class Pid:
    def __init__(self, pid_params):
        self.kp = pid_params[0]
        self.ki = pid_params[1]
        self.kd = pid_params[2]
        self.prev_e = 0.0

        self.i = 0.0

        self.prev_d_f = 0.0
        self.alpha = 0.2

    def calc(self, e, dt):
        if dt <= 0:
            return
        
        self.i += 0.5 * dt * (e + self.prev_e)

        d = (e - self.prev_e) / dt 

        d_f = self.alpha * self.prev_d_f + (1-self.alpha) * d

        u = self.kp * e + self.ki * self.i + self.kd * d_f

        self.prev_e = e
        self.prev_d_f = d_f
        return u