import numpy as np
import cv2

class WebcamCapturer:

    # Video capture
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    def getSingleFrame(self):
        ret, frame = self.cap.read()
        return frame

    def continuousDisplay(self):
        while(True):
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # Display the resulting frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def shutdown(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    capturer = WebcamCapturer()
    capturer.continuousDisplay()
    capturer.shutdown()
