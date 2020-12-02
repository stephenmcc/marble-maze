import cv2
import numpy as np;

def test_red_marble(index):
    image = cv2.imread("..\\..\\resources\\RedMarble" + str(index) + ".jpg")
    redLower = (20,25,110)
    redUpper = (55,60,150)

    # showImageAndWait(image)
    get_blob_keypoints(image, redLower, redUpper, getMarbleBlobDetectionParameters())

def test_blue_marble(index):
    image = cv2.imread("..\\..\\resources\\BlueMarble" + str(index) + ".jpg")
    blueLower = (55, 25, 20)
    blueUpper = (230, 100, 50)

    show_image_and_wait(image)
    get_blob_keypoints(image, blueLower, blueUpper, get_marble_blob_detection_parameters())

def get_blob_keypoints(image, lowerThreshold, upperThreshold, params):
    mask = cv2.inRange(image, lowerThreshold, upperThreshold)
    mask = cv2.erode(mask, None, iterations=0)
    mask = cv2.dilate(mask, None, iterations=0)
    image = cv2.bitwise_and(image, image, mask = mask)

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(mask)

    print("Number of keypoints: " + str(len(keypoints)))
    for i in range(len(keypoints)):
        print("Keypoint " + str(i))
        print("x = " + str(keypoints[i].pt[0]))
        print("y = " + str(keypoints[i].pt[1]))
        print("size = " + str(keypoints[i].size))

    image_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    image = cv2.bitwise_and(image, image_with_keypoints, mask = mask)

    show_image_and_wait(image)

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

def show_image_and_wait(image):
    cv2.imshow('IPWebcam',image)
    cv2.waitKey(0)

if __name__ == "__main__":
    test_blue_marble(1)
