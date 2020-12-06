import os
import cv2
import time

from blob_detector import OpenCVBlobDetector
from blob_detector import SimpleBlobDetector
from webcam_capturer import WebcamCapturer
from dynamixel_motor_driver import MotorDriver
from marble_state_manager import MarbleStateManager

def display_webcam():
    capturer = WebcamCapturer()

    while(True):
        # Capture frame-by-frame
        frame = capturer.getCroppedFrame()

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capturer.shutdown()

def save_current_image(file_name, delay):
    time.sleep(delay)
    capturer = WebcamCapturer()

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

def save_a_few_images(name_prefix, number_of_images):
    capturer = WebcamCapturer()

    # for some reason the first few images are bad
    for i in range(3):
        frame = capturer.getCroppedFrame()
        time.sleep(0.4)

    frames = []
    for i in range(number_of_images):
        frames.append(capturer.getCroppedFrame())
        time.sleep(0.2)

    capturer.shutdown()

    dirname = os.path.dirname(__file__)
    for i in range(len(frames)):
        filename = os.path.join(dirname, '..\\..\\..\\resources\\' + name_prefix + str(i) + '.png')
        cv2.imwrite(filename, frames[i])

def test_saved_image(index):
    image = cv2.imread("..\\..\\..\\resources\\Test" + str(index) + ".png")

    blueMarbleDetector = SimpleBlobDetector.createBlueMarbleDetector()

    marbleStateManager = MarbleStateManager()
    marbleStateManager.initialize(48, 294)

    filtered_image, blob = blueMarbleDetector.detectBlob1(image, marbleStateManager)

    print(str(blob[0]) + ", " + str(blob[1]))

    mark_detected_position(filtered_image, blob[0], blob[1])
    cv2.imshow('image', filtered_image)

    cv2.setMouseCallback('image', WebcamCapturer.mouse_callback)
    cv2.waitKey(0)

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

def display_simple_detection_results():
    capturer = WebcamCapturer()
    detector = SimpleBlobDetector.createBlueMarbleDetector()

    while(True):
        # Capture frame-by-frame
        frame = capturer.getCroppedFrame()
        filtered_image, blob = detector.detectBlob0(frame)
        mark_detected_position(filtered_image, blob[0], blob[1])

        cv2.imshow('frame', filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capturer.shutdown()

def track_marble():
    capturer = WebcamCapturer()
    detector = SimpleBlobDetector.createBlueMarbleDetector()

    # first couple frames are bad
    for i in range(3):
        frame = capturer.getCroppedFrame()
        time.sleep(0.2)

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
            mark_detected_position(filtered_image, marbleStateManager.x, marbleStateManager.y)
            detectedMarbleLastTick = True
        else:
            detectedMarbleLastTick = False

        print(str(marbleStateManager.x) + ', ' + str(marbleStateManager.y))
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

def go_to_setpoint():
    setpointX = 300
    setpointY = 400

    capturer = WebcamCapturer()
    detector = SimpleBlobDetector.createBlueMarbleDetector()
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
            mark_detected_position(filtered_image, marbleStateManager.x, marbleStateManager.y)
            mark_goal_position(filtered_image, setpointX, setpointY)
            detectedMarbleLastTick = True
        else:
            detectedMarbleLastTick = False

        dx = setpointX - marbleStateManager.x
        dy = setpointY - marbleStateManager.y
        vx = marbleStateManager.vx
        vy = marbleStateManager.vy

        kp = 0.12
        kd = 0.03

        signX = 1.0
        signY = -1.0

        ux = signX * (kp * dx - kd * vx)
        uy = signY * (kp * dy - kd * vy)

        motorDriverX.set_goal_relative_to_level(int(ux))
        motorDriverY.set_goal_relative_to_level(int(uy))

        print(str(marbleStateManager.x) + ', ' + str(marbleStateManager.y))
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


if __name__ == "__main__":
    # show unfiltered image
    # display_webcam()

    # write current webcam image to file
    image_name = "Test6.png"
    # save_current_image(image_name, 0.0)

    # save N images
    # save_a_few_images('Test', 20)

    # run detector on saved image
    # test_saved_image(6)

    # show live filtered image
    # display_simple_detection_results()

    # run full tracker
    # track_marble()

    # show motor min/max angle, etc.
    # print_motor_states()

    # static setpoint
    go_to_setpoint()
