import numpy as np
import cv2

WEBCAM_INDEX = 1

IMG_WIDTH = 1920
IMG_HEIGHT = 1080

MIN_X = 640
MAX_X = 1500

MIN_Y = 73
MAX_Y = 907

class WebcamCapturer:
    # Video capture
    cap = cv2.VideoCapture(WEBCAM_INDEX, cv2.CAP_DSHOW)

    # Has a double click happened
    has_clicked = False
    # Mouse click position
    mouse_click_x = -1
    mouse_click_y = -1

    def __init__(self):
        self.cap.set(3, IMG_WIDTH)
        self.cap.set(4, IMG_HEIGHT)

    def getSingleFrame(self):
        ret, frame = self.cap.read()
        return frame

    def getCroppedFrame(self):
        ret, frame = self.cap.read()
        return frame[MIN_Y:MAX_Y, MIN_X:MAX_X]

    def shutdown(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            print(str(x) + ", " + str(y))
            WebcamCapturer.mouse_click_x = x
            WebcamCapturer.mouse_click_y = y
            WebcamCapturer.has_clicked = True
