import os
import cv2
import time

from blob_detector import BlobDetector
from webcam_capturer import WebcamCapturer
from dynamixel_motor_driver import MotorDriver

def display_detection_results():
    capturer = WebcamCapturer()
    detector = BlobDetector.createBlueMarbleDetector()

    while(True):
        # Capture frame-by-frame
        frame = capturer.getSingleFrame()
        filtered_image, keypoints = detector.detectBlob(frame)
        cv2.imshow('frame', filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capturer.shutdown()

def save_current_image(file_name):
    capturer = WebcamCapturer()
    frame = capturer.getSingleFrame()

    # cv2.imshow('frame', frame)
    # cv2.waitKey(0)

    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '..\\..\\..\\resources\\' + file_name)
    cv2.imwrite(filename, frame)

    capturer.shutdown()

def print_motor_states():
    motorDriver1 = MotorDriver(10)
    motorDriver2 = MotorDriver(20)

    motorDriver1.print_current_state()
    motorDriver2.print_current_state()

    time.sleep(1)

    motorDriver1.shutdown()
    motorDriver2.shutdown()

if __name__ == "__main__":
    # show live filtered image
    # display_detection_results()

    # write current webcam image to file
    image_name = "Test2.jpg"
    save_current_image(image_name)

    # show motor min/max angle, etc.
    # print_motor_states()
