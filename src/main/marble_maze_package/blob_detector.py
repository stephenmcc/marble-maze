import cv2
import numpy as np;
from marble_state_manager import MarbleStateManager
from math_utils import *

# Coordinates are BGR
MARBLE_LOWER = (110, 80, 0)
MARBLE_UPPER = (230, 130, 80)

WALL_LOWER = (60, 145, 35)
WALL_UPPER = (120, 210, 90)

CORNER_LOWER = (60, 145, 35)
CORNER_UPPER = (120, 210, 90)

class SimpleBlobDetector:
    def __init__(self, lowerThreshold, upperThreshold):
        self.lowerThreshold = lowerThreshold
        self.upperThreshold = upperThreshold

    def createMarbleDetector():
        return SimpleBlobDetector(MARBLE_LOWER, MARBLE_UPPER)

    def createObstacleDetector():
        return SimpleBlobDetector(WALL_LOWER, WALL_UPPER)

    def applyFilter(self, image):
        mask = cv2.inRange(image, self.lowerThreshold, self.upperThreshold)
        filtered_image = cv2.bitwise_and(image, image, mask = mask)
        return filtered_image

    def detectBlob0(self, image):
        return self.detectBlobInternal(image, 0, len(image[0]) - 1, 0, len(image) - 1)

    def detectBlob1(self, image, marbleStateManager):
        maxSearchDistance = 30
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
                if (int(pixel[0]) + int(pixel[1]) + int(pixel[2]) != 0):
                    numberOfPixels += 1
                    centroidX += x
                    centroidY += y

        if (numberOfPixels == 0):
            return filtered_image, (-1, -1)
        else:
            return filtered_image, (centroidX / numberOfPixels, centroidY / numberOfPixels)
