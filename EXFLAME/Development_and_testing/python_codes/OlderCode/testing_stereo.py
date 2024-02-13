import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

#-----------------------------------------------------------------
def scale_image(img, scale):
    width = int(img.shape[1] * scale / 100)
    height = int(img.shape[0] * scale / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    return img
#-----------------------------------------------------------------

colour_image = cv.imread('left_image_8_adj.png')
# colour_image = scale_image(colour_image, 100)

imgL = cv.imread('left_image_8_adj.png', cv.IMREAD_GRAYSCALE)
imgR = cv.imread('right_image_8_adj.png', cv.IMREAD_GRAYSCALE)

stereo = cv.StereoBM_create(numDisparities=128, blockSize=13)
disparity = stereo.compute(imgL,imgR)

cv.imshow("Colour Image", colour_image)
# cv.imshow("Disparity Map", scale_image(disparity*255/16, 75))

plt.imshow(disparity,'gray')
plt.show()

# cv.waitKey(0)
cv.destroyAllWindows()
