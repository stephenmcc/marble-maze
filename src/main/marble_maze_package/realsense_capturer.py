import pyrealsense2 as rs
import numpy as np
import cv2

IMG_WIDTH = 640
IMG_HEIGHT = 480

MIN_X = 233
MAX_X = 578

MIN_Y = 50
MAX_Y = 389

class RealsenseCapturer:
    # Has a double click happened
    has_clicked = False
    # Mouse click position
    mouse_click_x = -1
    mouse_click_y = -1

    def __init__(self):
        config = rs.config()
        config.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, 30)
        self.pipeline = rs.pipeline()
        self.pipeline.start(config)

    def getSingleFrame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def getCroppedFrame(self):
        uncroppedFrame = self.getSingleFrame()
        return uncroppedFrame[MIN_Y:MAX_Y, MIN_X:MAX_X]

    def shutdown(self):
        # When everything done, release the capture
        self.pipeline.stop()

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            print(str(x) + ", " + str(y))
            RealsenseCapturer.mouse_click_x = x
            RealsenseCapturer.mouse_click_y = y
            RealsenseCapturer.has_clicked = True
