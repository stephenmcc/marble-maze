import time
from math_utils import *

class MarbleStateManager:

    # What percentage of new state to use
    alpha_position = 0.65

    def __init__(self):
        self.has_initialized = False
        self.last_timestamp = -1.0

    def initialize(self, x, y):
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.has_initialized = True

    def new_state_detected(self, x, y):
        t = time.time()
        newX = alpha_filter(self.alpha_position, self.x, x)
        newY = alpha_filter(self.alpha_position, self.y, y)
        if (self.last_timestamp > 0.0):
            dt = t - self.last_timestamp
            self.vx = (newX - self.x) / dt
            self.vy = (newY - self.y) / dt
        self.last_timestamp = t
        self.x = newX
        self.y = newY

    def get_position():
        return (self.x, self.y)

    def get_velocity():
        return (self.vx, self.vy)
