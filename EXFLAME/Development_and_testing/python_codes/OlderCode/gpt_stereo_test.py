import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

#-----------------------------------------------------------------
def scale_image(img, scale):
    width = int(img.shape[1] * scale / 100)
    height = int(img.shape[0] * scale / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    return img
#-----------------------------------------------------------------


colour_image = cv.imread('Images/Sample_Stereo/Art/view1.png')
# Load left and right images
left_image = cv.imread('Images/Sample_Stereo/Art/view1.png', cv.IMREAD_GRAYSCALE)
right_image = cv.imread('Images/Sample_Stereo/Art/view5.png', cv.IMREAD_GRAYSCALE)

ground_truth = cv.imread('Images\Sample_Stereo\Art\disp1.png', cv.IMREAD_GRAYSCALE)


num_disp = 224
blcok_size = 9

# Create a stereo block matching object
stereo = cv.StereoSGBM_create(
    minDisparity=0, 
    numDisparities=num_disp, 
    blockSize=blcok_size,
    P1=1 * 3 * blcok_size ** 2,
    P2=3 * 3 * blcok_size ** 2,
    #preFilterCap=63,          # Adjust this parameter
    #uniquenessRatio=1,       # Adjust this parameter
    speckleWindowSize=200,    # Adjust this parameter
    speckleRange=2 
    )

# Compute the disparity map
disparity_map = stereo.compute(left_image, right_image).astype(np.float32)/16


# Normalize the disparity map for display
normalized_disparity_map = cv.normalize(disparity_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)

focal_length_pixel = 3740

# print(disparity_map)
#disparity_map[np.where(disparity_map == -1)] = -9999999

depth_map = ((focal_length_pixel * 0.160) / (disparity_map+200)).astype(np.float32)


normalized_depth_map = cv.normalize(depth_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)

# Display the images and disparity map
cv.imshow('Left Image', scale_image(colour_image,80))
cv.imshow('Disparity Map', scale_image(normalized_disparity_map,80))
cv.imshow("depth map", scale_image(normalized_depth_map,80))
cv.waitKey(0)

fig1 = plt.figure(1)
plt.imshow(disparity_map,'gray')
fig2 = plt.figure(2)
plt.imshow(ground_truth,'gray')
fig3 = plt.figure(3)
plt.imshow(depth_map,'turbo_r')
plt.show()

cv.destroyAllWindows()


