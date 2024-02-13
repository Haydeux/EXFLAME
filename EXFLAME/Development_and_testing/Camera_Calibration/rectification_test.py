import cv2 as cv
import numpy as np

# Load calibration parameters from the text file
calibration_data = np.loadtxt('Camera_Calibration/stereoParams.txt',delimiter=',')

# Extract parameters
intrinsics_left = calibration_data[:3, :3]
distortion_left = calibration_data[3, :5]
intrinsics_right = calibration_data[4:7, :3]
distortion_right = calibration_data[7, :5]
extrinsics_R = calibration_data[8:11, :3]
extrinsics_T = calibration_data[11, :3]

# print(f"K_left:\n{intrinsics_left}\n\nK_right:\n{intrinsics_right}\n")
# print(f"dist_left:\n{distortion_left}\n\ndist_right:\n{distortion_right}\n")
# print(f"R:\n{extrinsics_R}\n\nT:\n{extrinsics_T}\n")

# Create stereo rectification maps
R1, R2, P1, P2, Q, roi_left, roi_right = cv.stereoRectify(
    intrinsics_left, distortion_left,
    intrinsics_right, distortion_right,
    (640, 480), # specify the size of your images
    extrinsics_R, extrinsics_T,
    flags=cv.CALIB_ZERO_DISPARITY, alpha=1
)

# Compute the rectification maps
map_left_x, map_left_y = cv.initUndistortRectifyMap(
    intrinsics_left, distortion_left, R1, P1,
    (640, 480), cv.CV_32FC1
)
map_right_x, map_right_y = cv.initUndistortRectifyMap(
    intrinsics_right, distortion_right, R2, P2,
    (640, 480), cv.CV_32FC1
)

# Load a new stereo image pair
left_image = cv.imread('Images/Images/Images/left/left_image_14.png')
right_image = cv.imread('Images/Images/Images/right/right_image_14.png')

cv.imshow('Left', left_image)
cv.imshow('Right', right_image)
cv.waitKey(10)


# Rectify the images
rectified_left = cv.remap(left_image, map_left_x, map_left_y, cv.INTER_LINEAR)
rectified_right = cv.remap(right_image, map_right_x, map_right_y, cv.INTER_LINEAR)


x,y,w,h = roi_left
cv.rectangle(rectified_left,(x,y),(x+w,y+h),(0,255,0),1)
for i in range(20):
    cv.line(rectified_left, (x+1,y+i*h//20+10),(x+w-1,y+i*h//20+10),(0,0,255),1)

x,y,w,h = roi_right
cv.rectangle(rectified_right,(x,y),(x+w,y+h),(0,255,0),1)
for i in range(20):
    cv.line(rectified_right, (x+1,y+i*h//20+10),(x+w-1,y+i*h//20+10),(0,0,255),1)

# Display rectified images side by side
cv.imshow('Rectified Left', rectified_left)
cv.imshow('Rectified Right', rectified_right)
cv.waitKey(0)
cv.destroyAllWindows()
