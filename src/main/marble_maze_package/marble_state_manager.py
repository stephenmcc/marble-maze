import time
from math_utils import *

class MarbleStateManager:

    # What percentage of new state to use
    alpha_position = 0.6

    # use velocity to predict next position
    velocity_multiplier = 0.6

    # keep track of update rate to use in controller
    dt = 0.0

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
        predictedX = self.x
        predictedY = self.y
        if (self.last_timestamp > 0.0):
            dt = t - self.last_timestamp
            predictedX = predictedX + self.velocity_multiplier * dt * self.vx
            predictedY = predictedY + self.velocity_multiplier * dt * self.vy
            self.dt = dt

        newX = alpha_filter(self.alpha_position, predictedX, x)
        newY = alpha_filter(self.alpha_position, predictedY, y)
        if (self.last_timestamp > 0.0):
            self.vx = (newX - self.x) / dt
            self.vy = (newY - self.y) / dt
        self.x = newX
        self.y = newY
        self.last_timestamp = t

    def get_position(self):
        return (self.x, self.y)

    def get_velocity(self):
        return (self.vx, self.vy)

    def get_dt(self):
        return self.dt
