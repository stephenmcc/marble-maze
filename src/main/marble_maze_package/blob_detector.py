import cv2
import numpy as np;
from marble_state_manager import MarbleStateManager
from math_utils import *

# Coordinates are BGR
MARBLE_LOWER = (110, 80, 0)
MARBLE_UPPER = (230, 120, 80)

WALL_LOWER = (60, 120, 35)
WALL_UPPER = (125, 220, 90)

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

    def initializeMarblePosition(self, image):
        mask = cv2.inRange(image, self.lowerThreshold, self.upperThreshold)
        filtered_image = cv2.bitwise_and(image, image, mask = mask)

        search_radius = 4

        cx = 0.0
        cy = 0.0
        w = 0.0

        for y in range(len(filtered_image)):
            for x in range(len(filtered_image[0])):
                pixel = filtered_image[y][x]
                in_range = int(pixel[0]) + int(pixel[1]) + int(pixel[2]) != 0
                if (in_range):
                    d = self.neighborhoodDensity(filtered_image, x, y, search_radius)
                    w += d
                    cx += d * x
                    cy += d * y
        assert (w > 0.0), "No valid pixels found within the marble filter's range"
        return filtered_image, (cx / w, cy / w)

    def neighborhoodDensity(self, image, x, y, search_radius):
        minX = max(x - search_radius, 0)
        maxX = min(x + search_radius, len(image[0]))
        minY = max(y - search_radius, 0)
        maxY = min(y + search_radius, len(image))

        count = 0
        total = 0

        for xi in range(minX, maxX):
            for yi in range(minY, maxY):
                total += 1
                pixel = image[yi][xi]
                in_range = int(pixel[0]) + int(pixel[1]) + int(pixel[2]) != 0
                if (in_range):
                    count += 1
        return float(count) / total

    def computeAverageThreshold(self, image):
        return self.computeAverageThresholdInternal(image, 0, len(image[0]) - 1, 0, len(image) - 1)

    def computeMarblePosition(self, image, marbleStateManager):
        maxSearchDistance = 30
        minX = max(0, int(marbleStateManager.x - maxSearchDistance) - 1)
        maxX = min(len(image[0]), int(marbleStateManager.x + maxSearchDistance))
        minY = max(0, int(marbleStateManager.y - maxSearchDistance) - 1)
        maxY = min(len(image), int(marbleStateManager.y + maxSearchDistance))
        return self.computeAverageThresholdInternal(image, minX, maxX, minY, maxY)

    def computeAverageThresholdInternal(self, image, minX, maxX, minY, maxY):
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
