from math_utils import *

class MarbleStateManager:

    # What percentage of new state to use
    alpha_position = 1.0

    def __init__(self):
        self.has_initialized = False

    def initialize(self, x, y):
        self.x = x
        self.y = y
        self.has_initialized = True

    def new_state_detected(self, x, y):
        self.x = alpha_filter(self.alpha_position, self.x, x)
        self.y = alpha_filter(self.alpha_position, self.y, y)
