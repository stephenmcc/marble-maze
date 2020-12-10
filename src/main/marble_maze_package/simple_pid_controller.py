from math_utils import *

# PID gains
KP = 0.85
KD = 0.5
KI = 2.0

# How positive motor motion maps to image coordinates
SIGN_X = 1.0
SIGN_Y = -1.0

MAX_INTEGRAL_ERROR = 120

# Exponential leak for integrated error, half-life of around 3 seconds at 30Hz
INTEGRAL_LEAK_RATIO = 0.9925

# If moving this fast towards goal, clear integral
VELOCITY_THRESHOLD_TO_CLEAR_INTEGRAL = 24.0

MAX_DELTA_CONTROL = 15

class SimplePIDController:
    # integrated error
    integrated_error_x = 0.0
    integrated_error_y = 0.0

    prev_ux = 0.0
    prev_uy = 0.0

    def __init__(self, marbleState):
        self.marbleState = marbleState

    def set_setpoint(self, setpoint):
        self.setpointX = setpoint[0]
        self.setpointY = setpoint[1]

    def do_control(self):
        dx = self.setpointX - self.marbleState.x
        dy = self.setpointY - self.marbleState.y
        vx = self.marbleState.vx
        vy = self.marbleState.vy

        self.integrated_error_x += self.marbleState.get_dt() * dx
        self.integrated_error_y += self.marbleState.get_dt() * dy

        self.integrated_error_x = clamp(self.integrated_error_x, MAX_INTEGRAL_ERROR)
        self.integrated_error_y = clamp(self.integrated_error_y, MAX_INTEGRAL_ERROR)

        if (signum(vx) == signum(self.integrated_error_x) and abs(vx) > VELOCITY_THRESHOLD_TO_CLEAR_INTEGRAL):
            self.integrated_error_x = 0.0
        if (signum(vy) == signum(self.integrated_error_y) and abs(vy) > VELOCITY_THRESHOLD_TO_CLEAR_INTEGRAL):
            self.integrated_error_y = 0.0

        print(str(self.integrated_error_x) + ", " + str(self.integrated_error_y))

        ux = SIGN_X * (KP * dx - KD * vx + KI * self.integrated_error_x)
        uy = SIGN_Y * (KP * dy - KD * vy + KI * self.integrated_error_y)

        dux = ux - self.prev_ux
        duy = uy - self.prev_uy

        dux = clamp(dux, MAX_DELTA_CONTROL)
        duy = clamp(duy, MAX_DELTA_CONTROL)

        ux_clamped = self.prev_ux + dux
        uy_clamped = self.prev_uy + duy

        self.prev_ux = ux_clamped
        self.prev_uy = uy_clamped

        self.integrated_error_x *= INTEGRAL_LEAK_RATIO
        self.integrated_error_y *= INTEGRAL_LEAK_RATIO

        return (ux_clamped, uy_clamped)
