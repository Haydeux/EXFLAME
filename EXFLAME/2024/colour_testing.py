import cv2 as cv
import numpy as np

# Dete
def colour_detect(image):
    # Convert the image to HSV color space
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # Define the range for detecting white in HSV
    lower_white = np.array([0, 0, 100])
    upper_white = np.array([180, 100, 200])

    # Create a mask for white regions
    mask = cv.inRange(hsv, lower_white, upper_white)

    # Find contours in the masked image
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    regions = []
    for contour in contours:
        # Get the bounding box for each contour
        x, y, w, h = cv.boundingRect(contour)
        x1, y1 = x, y
        x2, y2 = x + w, y + h
        regions.append(((x1, y1), (x2, y2)))

    # Optionally draw rectangles on the image for visualization
    for region in regions:
        cv.rectangle(image, region[0], region[1], (255, 0, 255), 3)
    cv.imshow("Image", image)
    cv.imshow("HSV", hsv)
    cv.imshow("Mask", mask)
    cv.waitKey(1)

    return regions


image = cv.imread("Git_clone/EXFLAME/2024/warm_up.png")

rectangles = colour_detect(image)

cv.waitKey(0)