import cv2
import numpy as np;
from marble_state_manager import MarbleStateManager
from math_utils import *

# Coordinates are BGR
RED_LOWER = (20,25,110)
RED_UPPER = (55,60,150)

BLUE_WC_LOWER = (70, 25, 20)
BLUE_WC_UPPER = (230, 75, 50)
BLUE_RS_LOWER = (110, 80, 0)
BLUE_RS_UPPER = (230, 130, 80)

GREEN_LOWER = (63, 95, 12)
GREEN_UPPER = (115, 155, 113)

class OpenCVBlobDetector:
    def __init__(self, params, lowerThreshold, upperThreshold):
        self.detector = cv2.SimpleBlobDetector_create(params)
        self.lowerThreshold = lowerThreshold
        self.upperThreshold = upperThreshold

    def createRedMarbleDetector():
        params = BlobDetector.get_marble_blob_detection_parameters()
        return OpenCVBlobDetector(params, RED_LOWER, RED_UPPER)

    def createBlueMarbleDetector():
        params = BlobDetector.get_marble_blob_detection_parameters()
        return OpenCVBlobDetector(params, BLUE_WC_LOWER, BLUE_WC_UPPER)

    def createGreenMarbleDetector():
        params = BlobDetector.get_marble_blob_detection_parameters()
        return OpenCVBlobDetector(params, GREEN_LOWER, GREEN_UPPER)

    def detectBlob(self, image):
        mask = cv2.inRange(image, self.lowerThreshold, self.upperThreshold)
        keypoints = self.detector.detect(mask)

        image_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        filtered_image = cv2.bitwise_and(image, image_with_keypoints, mask = mask)

        return filtered_image, keypoints

    def get_marble_blob_detection_parameters():
        params = cv2.SimpleBlobDetector_Params()

        # params.minDistBetweenBlobs = 50;
        #
        params.minThreshold = 100;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = False
        params.minArea = -1
        params.maxArea = 1500

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.01

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.1

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

    def get_corner_blob_detection_parameters():
        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 100;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = -1
        params.maxArea = 1500

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.01

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.1

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

class SimpleBlobDetector:
    def __init__(self, lowerThreshold, upperThreshold):
        self.lowerThreshold = lowerThreshold
        self.upperThreshold = upperThreshold

    def createBlueMarbleDetector():
        return SimpleBlobDetector(BLUE_WC_LOWER, BLUE_WC_UPPER)

    def createBlueMarbleDetectorRS():
        return SimpleBlobDetector(BLUE_RS_LOWER, BLUE_RS_UPPER)

    def createGreenMarbleDetector():
        return SimpleBlobDetector(GREEN_LOWER, GREEN_UPPER)

    def detectBlob0(self, image):
        return self.detectBlobInternal(image, 0, len(image[0]) - 1, 0, len(image) - 1)

    def detectBlob1(self, image, marbleStateManager):
        maxSearchDistance = 30 # 35
        minX = max(0, int(marbleStateManager.x - maxSearchDistance) - 1)
        maxX = min(len(image[0]), int(marbleStateManager.x + maxSearchDistance))
        minY = max(0, int(marbleStateManager.y - maxSearchDistance) - 1)
        maxY = min(len(image), int(marbleStateManager.y + maxSearchDistance))
        return self.detectBlobInternal(image, minX, maxX, minY, maxY)

    def detectBlobInternal(self, image, minX, maxX, minY, maxY):
        mask = cv2.inRange(image, self.lowerThreshold, self.upperThreshold)
        filtered_image = cv2.bitwise_and(image, image, mask = mask)

        numberOfPixels = 0
        centroidX = 0
        centroidY = 0

        for y in range(minY, maxY):
            row = filtered_image[y]
            for x in range(minX, maxX):
                pixel = row[x]
                if (int(pixel[0]) * int(pixel[1]) * int(pixel[2]) != 0):
                    numberOfPixels += 1
                    centroidX += x
                    centroidY += y

        if (numberOfPixels == 0):
            return filtered_image, (-1, -1)
        else:
            return filtered_image, (centroidX / numberOfPixels, centroidY / numberOfPixels)
