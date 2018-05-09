import math

def saturate(value, min_value, max_value):
    return max([min_value, min([max_value, value])])


class State(object):
    def __init__(self, x, dx):
        self.x = x
        self.dx = dx
        
    def __add__(self, other):
        return State(self.x + other.x, self.dx + other.dx)
      
    def __sub__(self, other):
        return State(self.x - other.x, self.dx - other.dx)


class PID(object):
    def __init__(self, kp, kd, ki):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._output_min = -1000000 # should be replace by a true inf
        self._output_max = 1000000
        self._integral_min = -1000000
        self._integral_max = 1000000
        self._integral_error = 0
        
    def output(self, desired_state, measured_state, dt):
        error_state = desired_state - measured_state
        self._integral_error = saturate(self._integral_error + (error_state.x * dt),
                                        self._integral_min,
                                        self._integral_max)
        
        output = (self._kp * error_state.x) + \
                 (self._kd * error_state.dx) + \
                 (self._ki * self._integral_error)
        return saturate(output, self._output_min, self._output_max)
      
    def set_output_saturation(self, min_value, max_value):
        self._output_min = min_value
        self._output_max = max_value
        
    def set_integral_saturation(self, min_value, max_value):
        self._integral_min = min_value
        self._integral_max = max_value
        
    def reset_integral(self, value = 0):
        self._integral_error = value
        
        
