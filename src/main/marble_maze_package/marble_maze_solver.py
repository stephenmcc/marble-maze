import os
import cv2
import time
import numpy as np
import datetime

from blob_detector import SimpleBlobDetector
from dynamixel_motor_driver import MotorDriver
from marble_state_manager import MarbleStateManager
from realsense_capturer import RealsenseCapturer
from simple_pid_controller import SimplePIDController
from list_of_setpoints import ListOfSetpoints
from math_utils import *

RECORD = True
positionLog = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(str(x) + ", " + str(y))
        MarbleMazeSolver.mouse_click_x = x
        MarbleMazeSolver.mouse_click_y = y
        MarbleMazeSolver.has_clicked = True

def mark_detected_position(image, blobX, blobY):
    width = 3
    for i in range(-width, width+1):
        for j in range(-width, width+1):
            image[int(blobY) + j, int(blobX) + i] = (50, 50, 180)

def mark_goal_position(image, blobX, blobY):
    width = 3
    for i in range(-width, width+1):
        for j in range(-width, width+1):
            image[int(blobY) + j, int(blobX) + i] = (50, 180, 40)

class MarbleMazeSolver:
    detected_marble_last_tick = False
    marbleStateManager = MarbleStateManager()
    controller = SimplePIDController(marbleStateManager)

    # Has a double click happened
    has_clicked = False
    # Mouse click position
    mouse_click_x = -1
    mouse_click_y = -1

    start_time = 0.0

    firstTick = True

    def __init__(self, setpointManager):
        self.capturer = RealsenseCapturer()
        self.detector = SimpleBlobDetector.createBlueMarbleDetectorRS()
        self.motorDriverX = MotorDriver.create_motor_x()
        self.motorDriverY = MotorDriver.create_motor_y()
        self.setpointManager = setpointManager

    def initialize(self):
        print('initializing...')
        self.enable_and_level()
        self.wait_for_user_to_specify_ball_position()
        self.setpointManager.on_start()
        self.controller.set_setpoint(self.setpointManager.get_setpoint())
        print('initialize complete')

    def update(self):
        try:
            return self.update_internal()
        except Exception as e:
            print (e)
            return False

    def update_internal(self):
        if (self.firstTick):
            self.start_time = time.time()
            self.firstTick = False
        setpoint = self.setpointManager.get_setpoint()
        self.controller.set_setpoint(setpoint)
        if (self.setpointManager.is_done()):
            return False

        frame = self.capturer.getCroppedFrame()
        if (self.detected_marble_last_tick):
            filtered_image, blob = self.detector.detectBlob1(frame, self.marbleStateManager)
        else:
            filtered_image, blob = self.detector.detectBlob0(frame)

        if (blob[0] != -1):
            if (self.detected_marble_last_tick):
                self.marbleStateManager.new_state_detected(blob[0], blob[1])
            else:
                self.marbleStateManager.initialize(blob[0], blob[1])
            mark_detected_position(filtered_image, self.marbleStateManager.x, self.marbleStateManager.y)
            mark_goal_position(filtered_image, setpoint[0], setpoint[1])
            self.detected_marble_last_tick = True
        else:
            self.detected_marble_last_tick = False

        u = self.controller.do_control()
        self.motorDriverX.set_goal_relative_to_level(int(u[0]))
        self.motorDriverY.set_goal_relative_to_level(int(u[1]))

        if (RECORD):
            t = time.time() - self.start_time
            x = self.marbleStateManager.x
            y = self.marbleStateManager.y
            positionLog.append((t, x, y, setpoint[0], setpoint[1]))

        cv2.imshow('frame', filtered_image)
        return not (cv2.waitKey(1) & 0xFF == ord('q'))

    def enable_and_level(self):
        self.motorDriverX.enable_torque()
        self.motorDriverY.enable_torque()

        self.motorDriverX.go_to_level()
        self.motorDriverY.go_to_level()

    def wait_for_user_to_specify_ball_position(self):
        for i in range(5):
            frame = self.capturer.getCroppedFrame()
            time.sleep(0.3)
        frame = self.capturer.getCroppedFrame()
        cv2.imshow('image', frame)
        cv2.setMouseCallback('image', mouse_callback)
        print('Double click on the marble\'s position')
        while(True):
            if (MarbleMazeSolver.has_clicked or (cv2.waitKey(1) & 0xFF == ord('q'))):
                break
        self.marbleStateManager.initialize(float(MarbleMazeSolver.mouse_click_x), float(MarbleMazeSolver.mouse_click_y))
        self.detected_marble_last_tick = True

    def shutdown(self):
        print('Shutting down...')
        self.capturer.shutdown()
        self.motorDriverX.disable_torque()
        self.motorDriverY.disable_torque()
        time.sleep(0.1)
        self.motorDriverX.shutdown()
        self.motorDriverY.shutdown()

        if (RECORD):
            filename_prefix = datetime.datetime.now().strftime("%y%m%d%H%M%S")

            file = open("..\\..\\..\\..\\marble-maze-logs\\" + filename_prefix + "_Log.txt", "x")
            file.write("Time (seconds), X Position, Y Position, X Setpoint, Y Setpoint\n")
            for i in range(len(positionLog)):
                pos = positionLog[i]
                logString = ""
                for j in range(len(pos)):
                    logString += str(pos[j])
                    if (j != len(pos) - 1):
                        logString += ","
                logString += "\n"
                file.write(logString)
            file.close()

        print('Shutdown complete')

def run_solver(solver):
    solver.initialize()
    while(True):
        if (not solver.update()):
            break
    solver.shutdown()

if __name__ == "__main__":
    setpointManager = ListOfSetpoints()

    solver = MarbleMazeSolver(setpointManager)
    run_solver(solver)
