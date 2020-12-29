
class StaticSetpoint:

    def __init__(self, x, y):
        self.setpoint = (x, y)

    def on_start(self, goal, frame):
        pass

    def get_setpoint(self):
        return self.setpoint

    def is_done(self):
        return False
