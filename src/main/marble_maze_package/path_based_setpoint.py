from math_utils import *
from path_planner import PathPlanner
from blob_detector import *

LOOK_AHEAD_VAL = 3

class PathBasedSetpoint:
    def __init__(self, marbleStateManager):
        self.marbleStateManager = marbleStateManager
        self.obstacleDetector = SimpleBlobDetector.createObstacleDetector()

    def on_start(self, goal, frame):
        self.goal = goal
        filtered_image = self.obstacleDetector.applyFilter(frame)

        planner = PathPlanner()
        self.path, expanded, found_goal = planner.planPath(filtered_image, self.marbleStateManager.get_position(), goal)
        print("Got a path of length: " + str(len(self.path)))

    def get_setpoint(self):
        closest_distance = 100.0
        closest_index = -1
        marble_position = self.marbleStateManager.get_position()

        for i in range(len(self.path)):
            point = self.path[i]
            distance = distance2D(marble_position, point)
            if (distance < closest_distance):
                closest_distance = distance
                closest_index = i

        look_ahead_index = min(len(self.path) - 1, closest_index + LOOK_AHEAD_VAL)
        self.look_ahead_point = self.path[look_ahead_index]
        return self.look_ahead_point

    def is_done(self):
        if (self.look_ahead_point is None):
            return False
        return distance2D(self.goal, self.look_ahead_point) < 12
