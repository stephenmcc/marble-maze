import os
import traceback
import sys

import cv2
import time
import numpy as np
import datetime

from blob_detector import SimpleBlobDetector
from dynamixel_motor_driver import MotorDriver
from marble_state_manager import MarbleStateManager
from realsense_capturer import RealsenseCapturer
from simple_pid_controller import SimplePIDController
from path_planner import PathPlanner
from math_utils import *
from behavior_tools import *
from gui_tools import *

from pynput import mouse
from pynput import keyboard

WINDOW_NAME = "RobLabUI"

######### Common node objects #########
GOAL = (20, 320)

class WhiteBoard:
    ABORT = False
    PAUSED = False
    OPERATOR_MODE = False

    RAW_FRAME = None
    PATH = None
    FRAME_TO_SHOW = None

    OP_X = -1
    OP_Y = -1
    SETPOINT = (-1, -1)

############# HID Callbacks #############
def on_release(key):
    if (key == keyboard.Key.space):
        WhiteBoard.PAUSED = not WhiteBoard.PAUSED
        print("Paused: " + str(WhiteBoard.PAUSED))
    if (key == keyboard.Key.ctrl):
        WhiteBoard.OPERATOR_MODE = not WhiteBoard.OPERATOR_MODE
        print("Operator mode: " + str(WhiteBoard.OPERATOR_MODE))
    if (key == keyboard.Key.esc):
        WhiteBoard.ABORT = True

keyboard.Listener(on_release=on_release).start()

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("New setpoint: " + str(x) + ", " + str(y))
        WhiteBoard.OP_X = x
        WhiteBoard.OP_Y = y

############# Behavior Tree States #############
class Pause:
    def tick(self):
        if (WhiteBoard.PAUSED):
            return RUNNING
        else:
            return FAILURE

class Perceive:
    def __init__(self, marbleStateManager):
        self.marbleStateManager = marbleStateManager
        self.capturer = RealsenseCapturer()
        self.marble_detector = SimpleBlobDetector.createMarbleDetector()
        self.initialize = True
        self.first_tick = True

    def tick(self):
        if (self.first_tick):
            for i in range(5):
                self.capturer.getCroppedFrame()
                time.sleep(0.2)
                self.first_tick = False

        if (self.initialize):
            filtered_image, blob = self.marble_detector.initializeMarblePosition(WhiteBoard.RAW_FRAME)
            mark_cell(filtered_image, blob[0], blob[1], RED, 3)
            show_and_wait(filtered_image)
            self.marbleStateManager.initialize(blob[0], blob[1])
        else:
            filtered_image, blob = self.marble_detector.computeMarblePosition(WhiteBoard.RAW_FRAME, self.marbleStateManager)
            self.marbleStateManager.new_state_detected(blob[0], blob[1])

        self.initialize = blob[0] == -1
        if (self.initialize):
            return FAILURE
        else:
            return SUCCESS

    def shutdown(self):
        self.capturer.shutdown()

class ValidPath:
    def tick(self):
        if (WhiteBoard.PATH is None):
            return FAILURE
        return SUCCESS

class PlanPath:
    def __init__(self, marbleStateManager):
        self.marbleStateManager = marbleStateManager
        self.obstacle_detector = SimpleBlobDetector.createObstacleDetector()
        self.planner = PathPlanner()

    def tick(self):
        filtered_image = self.obstacle_detector.applyFilter(WhiteBoard.RAW_FRAME)
        WhiteBoard.PATH, expanded, found_goal = self.planner.planPath(filtered_image, self.marbleStateManager.get_position(), GOAL)
        print("Got a path of length: " + str(len(WhiteBoard.PATH)) + ", found goal: " + str(found_goal))

        mark_path(WhiteBoard.PATH, WhiteBoard.FRAME_TO_SHOW)
        show_and_wait(WhiteBoard.FRAME_TO_SHOW)

        if (found_goal):
            return SUCCESS
        else:
            return FAILURE

class FollowPath:
    def __init__(self, marbleStateManager):
        self.marbleStateManager = marbleStateManager
        self.controller = SimplePIDController(marbleStateManager)
        self.motorDriverX = MotorDriver.create_motor_x()
        self.motorDriverY = MotorDriver.create_motor_y()
        self.initialize = True

    def tick(self):
        if (self.initialize):
            self.motorDriverX.enable_torque()
            self.motorDriverY.enable_torque()
            self.motorDriverX.go_to_level()
            self.motorDriverY.go_to_level()
            self.initialize = False

        setpoint = self.compute_setpoint()
        self.controller.set_setpoint(setpoint)
        u = self.controller.do_control()
        self.motorDriverX.set_goal_relative_to_level(int(u[0]))
        self.motorDriverY.set_goal_relative_to_level(int(u[1]))

        cv2.imshow(WINDOW_NAME, WhiteBoard.RAW_FRAME)
        cv2.waitKey(1)

        return SUCCESS

    def shutdown(self):
        self.motorDriverX.disable_torque()
        self.motorDriverY.disable_torque()
        time.sleep(0.1)
        self.motorDriverX.shutdown()
        self.motorDriverY.shutdown()

    def compute_setpoint(self):
        if (WhiteBoard.OPERATOR_MODE):
            if (WhiteBoard.OP_X > 0):
                return (WhiteBoard.OP_X, WhiteBoard.OP_Y)
            elif (WhiteBoard.SETPOINT[0] > 0):
                return WhiteBoard.SETPOINT
            else:
                return self.marbleStateManager.get_position()

        look_ahead_val = 3
        closest_distance = 100.0
        closest_index = -1
        marble_position = self.marbleStateManager.get_position()

        for i in range(len(WhiteBoard.PATH)):
            point = WhiteBoard.PATH[i]
            distance = distance2D(marble_position, point)
            if (distance < closest_distance):
                closest_distance = distance
                closest_index = i

        look_ahead_index = min(len(WhiteBoard.PATH) - 1, closest_index + look_ahead_val)
        look_ahead_point = WhiteBoard.PATH[look_ahead_index]

        #### Update whiteboard
        WhiteBoard.SETPOINT = look_ahead_point
        WhiteBoard.OP_X = -1
        WhiteBoard.OP_Y = -1

        return look_ahead_point

############ Window management ############
def show_and_wait(image):
    cv2.imshow(WINDOW_NAME, image)
    cv2.waitKey(0)

if __name__ == "__main__":
    marbleStateManager = MarbleStateManager()

    pauseState = Pause()
    perceiveState = Perceive(marbleStateManager)
    validPathState = ValidPath()
    planPathState = PlanPath(marbleStateManager)
    followPathState = FollowPath(marbleStateManager)

    root = FallbackNode()
    root.addChild(pauseState)

    sequence0 = SequenceNode()
    root.addChild(sequence0)

    sequence0.addChild(perceiveState)

    pathFallback = FallbackNode()
    pathFallback.addChild(validPathState)
    pathFallback.addChild(planPathState)
    sequence0.addChild(pathFallback)

    sequence0.addChild(followPathState)

    while (not WhiteBoard.ABORT):
        try:
            WhiteBoard.RAW_FRAME = self.capturer.getCroppedFrame()
            WhiteBoard.FRAME_TO_SHOW = np.copy(WhiteBoard.RAW_FRAME)

            root.tick()
        except Exception as e:
            print(traceback.format_exc())
            break

    perceiveState.shutdown()
    followPathState.shutdown()
