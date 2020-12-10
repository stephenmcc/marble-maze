import time

class ListOfSetpoints:

    # hard-coded setpoints to go to
    setpoints = []
    setpoints.append((200, 200))
    setpoints.append((80, 175))
    setpoints.append((150, 100))
    setpoints.append((240, 260))

    # current index of setpoint
    setpointIndex = 0

    # how often to switch setpoint
    timePerSetpoint = 6.0

    # True after completing timePerSetpoint at each setpoint
    isDone = False

    def __init__(self):
        self.setpointX = self.setpoints[0][0]
        self.setpointY = self.setpoints[0][1]

    def on_start(self):
        self.start_time = time.time()

    def get_setpoint(self):
        t = time.time()
        timeInSetpoint = t - self.start_time
        if (timeInSetpoint > self.timePerSetpoint):
            if (self.setpointIndex + 1 >= len(self.setpoints)):
                self.isDone = True
                return (self.setpointX, self.setpointY)
            self.setpointIndex += 1
            self.start_time = t
            self.setpointX = self.setpoints[self.setpointIndex][0]
            self.setpointY = self.setpoints[self.setpointIndex][1]

        return (self.setpointX, self.setpointY)

    def is_done(self):
        return self.isDone
