import cv2
import numpy as np;

class BlobDetector:
    def __init__(self, params, lowerThreshold, upperThreshold):
        self.detector = cv2.SimpleBlobDetector_create(params)
        self.lowerThreshold = lowerThreshold
        self.upperThreshold = upperThreshold

    def createRedMarbleDetector():
        params = BlobDetector.get_marble_blob_detection_parameters()
        redLower = (20,25,110)
        redUpper = (55,60,150)
        return BlobDetector(params, redLower, redUpper)

    def createBlueMarbleDetector():
        params = BlobDetector.get_marble_blob_detection_parameters()
        blueLower = (55, 25, 20)
        blueUpper = (230, 100, 50)
        return BlobDetector(params, blueLower, blueUpper)

    def detectBlob(self, image):
        mask = cv2.inRange(image, self.lowerThreshold, self.upperThreshold)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, None, iterations=0)
        keypoints = self.detector.detect(mask)

        image_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        filtered_image = cv2.bitwise_and(image, image_with_keypoints, mask = mask)

        return filtered_image, keypoints

    def get_marble_blob_detection_parameters():
        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 100;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = -1
        params.maxArea = 1500

        # Filter by Circularity
        params.filterByCircularity = True
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

def test_red_marble_image(index):
    image = cv2.imread("..\\..\\resources\\RedMarble" + str(index) + ".jpg")
    redMarbleDetector = BlobDetector.createRedMarbleDetector()
    filtered_image, keypoints = redMarbleDetector.detectBlob(image)

    # showImageAndWait(image)
    show_detection_results(filtered_image, keypoints)

def test_blue_marble_image(index):
    image = cv2.imread("..\\..\\resources\\BlueMarble" + str(index) + ".jpg")
    blueMarbleDetector = BlobDetector.createBlueMarbleDetector()
    filtered_image, keypoints = blueMarbleDetector.detectBlob(image)

    # showImageAndWait(image)
    show_detection_results(filtered_image, keypoints)

def show_detection_results(image, keypoints):
    print("Number of keypoints: " + str(len(keypoints)))
    for i in range(len(keypoints)):
        print("Keypoint " + str(i))
        print("x = " + str(keypoints[i].pt[0]))
        print("y = " + str(keypoints[i].pt[1]))
        print("size = " + str(keypoints[i].size))

    cv2.imshow('FilteredImage', image)
    cv2.waitKey(0)

if __name__ == "__main__":
    # test_red_marble_image(1)
    test_blue_marble_image(1)
