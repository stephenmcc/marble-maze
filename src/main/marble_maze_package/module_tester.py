import os
import cv2
import time
import numpy as np

from blob_detector import SimpleBlobDetector
from dynamixel_motor_driver import MotorDriver
from marble_state_manager import MarbleStateManager
from realsense_capturer import RealsenseCapturer
from math_utils import *
from path_planner import *

# Colors for marking graphs
RED = (50, 50, 180)
YELLOW = (0, 175, 205)
GREEN = (50, 180, 40)
GREY = (50, 50, 50)

def display_camera(capturer):
    while(True):
        # Capture frame-by-frame
        frame = capturer.getCroppedFrame()

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capturer.shutdown()

def save_current_image(file_name, capturer):
    # for some reason the first few images are bad
    for i in range(3):
        frame = capturer.getCroppedFrame()
        time.sleep(0.4)

    frame = capturer.getCroppedFrame()
    # cv2.imshow('frame', frame)
    # cv2.waitKey(0)

    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '..\\..\\..\\resources\\' + file_name)
    cv2.imwrite(filename, frame)

    capturer.shutdown()

def save_a_few_images(capturer, name_prefix, number_of_images):
    # for some reason the first few images are bad
    print("Capturing at 0...")
    for i in range(40):
        print(str(i))
        frame = capturer.getCroppedFrame()

    frames = []
    for i in range(number_of_images):
        frames.append(np.copy(capturer.getCroppedFrame()))

    capturer.shutdown()

    dirname = os.path.dirname(__file__)
    for i in range(len(frames)):
        filename = os.path.join(dirname, '..\\..\\..\\resources\\' + name_prefix + str(i) + '.png')
        cv2.imwrite(filename, frames[i])

def test_saved_images(detector, name, start_index, end_index):
    for i in range(end_index):
        if (i < start_index):
            continue
        image = cv2.imread("..\\..\\..\\resources\\" + name + str(i) + ".png")
        filtered_image, blob = detector.detectBlob0(image)
        print(str(blob[0]) + ", " + str(blob[1]))
        # mark_cell(filtered_image, blob[0], blob[1], RED)
        cv2.imshow('image', filtered_image)
        cv2.waitKey(0)

def test_specific_image(detector, image_name, save_name):
    image = cv2.imread("..\\..\\..\\resources\\" + image_name + ".png")
    filtered_image, blob = detector.detectBlob0(image)
    dirname = os.path.dirname(__file__)

    filename = os.path.join(dirname, '..\\..\\..\\resources\\' + save_name + '.png')
    cv2.imwrite(filename, filtered_image)

    cv2.imshow('image', filtered_image)
    cv2.waitKey(0)

def mark_cell(image, blobX, blobY, color):
    width = 3
    for i in range(-width, width+1):
        for j in range(-width, width+1):
            image[int(blobY) + j, int(blobX) + i] = color

def display_simple_detection_results():
    capturer = RealsenseCapturer()
    detector = SimpleBlobDetector.createBlueMarbleDetector()

    while(True):
        # Capture frame-by-frame
        frame = capturer.getCroppedFrame()
        filtered_image, blob = detector.detectBlob0(frame)
        mark_cell(filtered_image, blob[0], blob[1], RED)

        cv2.imshow('frame', filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capturer.shutdown()

def track_marble(capturer):
    detector = SimpleBlobDetector.createBlueMarbleDetectorRS()

    # first couple frames are bad
    for i in range(40):
        frame = capturer.getCroppedFrame()
        # time.sleep(0.2)

    frame = capturer.getCroppedFrame()

    cv2.imshow('image', frame)
    cv2.setMouseCallback('image', WebcamCapturer.mouse_callback)

    while(True):
        if (WebcamCapturer.has_clicked or (cv2.waitKey(1) & 0xFF == ord('q'))):
            break

    marbleStateManager = MarbleStateManager()
    marbleStateManager.initialize(float(WebcamCapturer.mouse_click_x), float(WebcamCapturer.mouse_click_y))
    detectedMarbleLastTick = True

    while(True):
        frame = capturer.getCroppedFrame()
        if (detectedMarbleLastTick):
            filtered_image, blob = detector.detectBlob1(frame, marbleStateManager)
        else:
            filtered_image, blob = detector.detectBlob0(frame)

        if (blob[0] != -1):
            marbleStateManager.new_state_detected(blob[0], blob[1])
            mark_cell(filtered_image, marbleStateManager.x, marbleStateManager.y, RED)
            detectedMarbleLastTick = True
            print(str(marbleStateManager.x) + ', ' + str(marbleStateManager.y))
        else:
            detectedMarbleLastTick = False
            print("No marble detected")

        cv2.imshow('frame', filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capturer.shutdown()

def print_motor_states():
    motorDriverX = MotorDriver(10, 0)
    motorDriverY = MotorDriver(20, 0)

    motorDriverX.print_current_state()
    motorDriverY.print_current_state()

    time.sleep(1)

    motorDriverX.shutdown()
    motorDriverY.shutdown()

def go_to_setpoint(capturer):
    setpointX = 200
    setpointY = 200

    detector = SimpleBlobDetector.createBlueMarbleDetectorRS()
    motorDriverX = MotorDriver.create_motor_x()
    motorDriverY = MotorDriver.create_motor_y()

    motorDriverX.enable_torque()
    motorDriverY.enable_torque()

    motorDriverX.go_to_level()
    motorDriverY.go_to_level()

    # first couple frames are bad
    for i in range(3):
        frame = capturer.getCroppedFrame()
        time.sleep(0.2)

    frame = capturer.getCroppedFrame()
    time.sleep(0.2)

    cv2.imshow('image', frame)
    cv2.setMouseCallback('image', WebcamCapturer.mouse_callback)

    while(True):
        if (WebcamCapturer.has_clicked or (cv2.waitKey(1) & 0xFF == ord('q'))):
            break

    marbleStateManager = MarbleStateManager()
    marbleStateManager.initialize(float(WebcamCapturer.mouse_click_x), float(WebcamCapturer.mouse_click_y))
    detectedMarbleLastTick = True

    while(True):
        frame = capturer.getCroppedFrame()
        if (detectedMarbleLastTick):
            filtered_image, blob = detector.detectBlob1(frame, marbleStateManager)
        else:
            filtered_image, blob = detector.detectBlob0(frame)

        if (blob[0] != -1):
            if (detectedMarbleLastTick):
                marbleStateManager.new_state_detected(blob[0], blob[1])
            else:
                marbleStateManager.initialize(blob[0], blob[1])
            mark_cell(filtered_image, marbleStateManager.x, marbleStateManager.y, RED)
            mark_cell(filtered_image, setpointX, setpointY, GREEN)
            detectedMarbleLastTick = True
        else:
            detectedMarbleLastTick = False

        dx = setpointX - marbleStateManager.x
        dy = setpointY - marbleStateManager.y
        vx = marbleStateManager.vx
        vy = marbleStateManager.vy

        kp = 0.7
        kd = 0.5

        signX = 1.0
        signY = -1.0

        ux = signX * (kp * dx - kd * vx)
        uy = signY * (kp * dy - kd * vy)

        if (abs(vx) < 10 and abs(vy) < 10 and abs(dx) > 20 and abs(dy) > 20):
            ux = ux + signX * signum(dx) * 20
            uy = uy + signY * signum(dy) * 20

        ux = clamp(ux, 70)
        uy = clamp(uy, 70)

        motorDriverX.set_goal_relative_to_level(int(ux))
        motorDriverY.set_goal_relative_to_level(int(uy))

        print(str(dx) + ', ' + str(dy) + ', ' + str(marbleStateManager.vx) + ', ' + str(marbleStateManager.vy))
        cv2.imshow('frame', filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    time.sleep(1.0)
    motorDriverX.disable_torque()
    motorDriverY.disable_torque()
    time.sleep(0.1)

    motorDriverX.shutdown()
    motorDriverY.shutdown()
    capturer.shutdown()

def planPath(detector, image_name):
    image = cv2.imread("..\\..\\..\\resources\\" + image_name + ".png")
    filtered_image, blob = detector.detectBlob0(image)

    planner = PathPlanner()

    start = (330, 330)
    goal = (30, 330)

    path, expanded = planner.planPath(filtered_image, start, goal)

    print("Got a path of length: " + str(len(path)))

    # for i in range(len(expanded[0])):
    #     for j in range(len(expanded)):
    #         if (expanded[j][i]):
    #             imgCoord = binCoordToImageSpace((i, j))
    #             mark_cell(filtered_image, imgCoord[0], imgCoord[1], GREY)

    for i in range(len(path)):
        mark_cell(filtered_image, path[i][0], path[i][1], YELLOW)

    mark_cell(filtered_image, start[0], start[1], GREEN)
    mark_cell(filtered_image, goal[0], goal[1], RED)

    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '..\\..\\..\\resources\\planner_result.png')
    cv2.imwrite(filename, filtered_image)

    cv2.imshow('image', filtered_image)
    cv2.waitKey(0)

if __name__ == "__main__":
    # capturer = WebcamCapturer()
    capturer = RealsenseCapturer()

    detector = SimpleBlobDetector.createMarbleDetector()
    # detector = SimpleBlobDetector.createObstacleDetector()

    # show cropped, unfiltered image
    # display_camera(capturer)

    # write current webcam image to file
    image_name = "Maze_Up.png"
    # save_current_image(image_name, capturer)

    # save N images
    # save_a_few_images(capturer, 'Test', 20)

    # run detector on saved image
    test_saved_images(detector, "Test", 3, 20)

    # run detector on specific image
    # test_specific_image(detector, "MazeAndMarble", "MarbleAndMaze_filt1")

    # show live filtered image
    # display_simple_detection_results()

    # run full tracker
    # track_marble(capturer)

    # show motor min/max angle, etc.
    # print_motor_states()

    # static setpoint
    # go_to_setpoint(capturer)

    # test A* planner
    # planPath(detector, "MazeParts3")
