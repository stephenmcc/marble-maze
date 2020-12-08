from math_utils import *

# PID gains
KP = 0.7
KD = 0.5
KI = 0.5

# How positive motor motion maps to image coordinates
SIGN_X = 1.0
SIGN_Y = -1.0

MAX_INTEGRAL_ERROR = 75

# Exponential leak for integrated error, half-life of around 3 seconds at 30Hz
INTEGRAL_LEAK_RATIO = 0.9925

class SimplePIDController:
    # integrated error
    integrated_error_x = 0.0
    integrated_error_y = 0.0

    def __init__(self, marbleState):
        self.marbleState = marbleState

    def set_setpoint(self, x, y):
        self.setpointX = x
        self.setpointY = y
        self.initialized = True
        self.integrated_error_x = 0.0
        self.integrated_error_y = 0.0

    def do_control(self):
        if (not self.initialized):
            return (0, 0)

        dx = self.setpointX - self.marbleState.x
        dy = self.setpointY - self.marbleState.y
        vx = self.marbleState.vx
        vy = self.marbleState.vy

        self.integrated_error_x += self.marbleState.get_dt() * dx
        self.integrated_error_y += self.marbleState.get_dt() * dy
        self.integrated_error_x = clamp(self.integrated_error_x, MAX_INTEGRAL_ERROR)
        self.integrated_error_y = clamp(self.integrated_error_y, MAX_INTEGRAL_ERROR)

        ux = SIGN_X * (KP * dx - KD * vx + KI * self.integrated_error_x)
        uy = SIGN_Y * (KP * dy - KD * vy + KI * self.integrated_error_y)

        self.integrated_error_x *= INTEGRAL_LEAK_RATIO
        self.integrated_error_y *= INTEGRAL_LEAK_RATIO

        return (ux, uy)
