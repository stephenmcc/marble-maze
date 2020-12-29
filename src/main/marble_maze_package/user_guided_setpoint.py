import cv2

class UserGuidedSetpoint:

    has_clicked = False
    mouse_click_x = -1
    mouse_click_y = -1

    def __init__(self, marbleStateManager):
        self.marbleStateManager = marbleStateManager
        self.has_clicked = False

    def on_start(self, goal, frame):
        self.setpoint = self.marbleStateManager.get_position()
        return

    def get_setpoint(self):
        if (UserGuidedSetpoint.has_clicked):
            self.setpoint = (UserGuidedSetpoint.mouse_click_x, UserGuidedSetpoint.mouse_click_y)
            UserGuidedSetpoint.has_clicked = False
        return self.setpoint

    def is_done(self):
        return False

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("New setpoint: " + str(x) + ", " + str(y))
            UserGuidedSetpoint.mouse_click_x = x
            UserGuidedSetpoint.mouse_click_y = y
            UserGuidedSetpoint.has_clicked = True
