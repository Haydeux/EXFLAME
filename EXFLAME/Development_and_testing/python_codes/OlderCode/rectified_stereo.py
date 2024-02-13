import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from pypylon import pylon
from time import perf_counter

def get_rectifier_maps(file_name, a=0):
    # Load calibration parameters from the text file
    calibration_data = np.loadtxt(file_name, delimiter=',')

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
        flags=cv.CALIB_ZERO_DISPARITY, alpha=a
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
    
    return map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right



def main():
    # map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right = get_rectifier_maps('Camera_Code/Camera_Calibration/stereoParams.txt', 1)

    # Load a new stereo image pair
    # left_image = cv.imread('Images/Images/Images/left/left_image_14.png')
    # right_image = cv.imread('Images/Images/Images/right/right_image_14.png')

    # cv.imshow('Left', left_image)
    # cv.imshow('Right', right_image)
    # cv.waitKey(10)

    # # Rectify the images
    # rectified_left = cv.remap(left_image, map_left_x, map_left_y, cv.INTER_LINEAR)
    # rectified_right = cv.remap(right_image, map_right_x, map_right_y, cv.INTER_LINEAR)

    # x,y,w,h = roi_left
    # cv.rectangle(rectified_left,(x,y),(x+w,y+h),(0,255,0),1)
    # for i in range(20):
    #     cv.line(rectified_left, (x+1,y+i*h//20+10),(x+w-1,y+i*h//20+10),(0,0,255),1)

    # x,y,w,h = roi_right
    # cv.rectangle(rectified_right,(x,y),(x+w,y+h),(0,255,0),1)
    # for i in range(20):
    #     cv.line(rectified_right, (x+1,y+i*h//20+10),(x+w-1,y+i*h//20+10),(0,0,255),1)

    # # Display rectified images side by side
    # cv.imshow('Rectified Left', rectified_left)
    # cv.imshow('Rectified Right', rectified_right)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    map_left_x, map_left_y, map_right_x, map_right_y, _, _ = get_rectifier_maps('Camera_Code/Camera_Calibration/stereoParams.txt', 0)

    num_disp = 64
    block_size = 17

    # Create a stereo block matching object
    stereo = cv.StereoSGBM_create(
        minDisparity=1, 
        numDisparities=num_disp, 
        blockSize=block_size,
        P1=3 * 3 * block_size ** 2,
        P2=5 * 3 * block_size ** 2,
        #preFilterCap=63,          # Adjust this parameter
        #uniquenessRatio=10,       # Adjust this parameter
        speckleWindowSize=200,    # Adjust this parameter
        speckleRange=2 
        )

    focal_length_pixel = 855
    baseline = 0.03316 # meters
    baseline_mm = baseline*1000 # mm

    # Get all available devices
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    if not devices:
        print("No cameras found.")
        return

    # Create and open the first camera
    camera1 = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[0]))
    camera1.Open()

    # Create and open the second camera
    camera2 = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[1]))
    camera2.Open()

    try:
        # Set camera parameters for camera1 if needed
        # camera1.ExposureTime.SetValue(10000)

        # Set camera parameters for camera2 if needed
        # camera2.ExposureTime.SetValue(10000)

        # Start grabbing images for camera1
        camera1.StartGrabbing()

        # Start grabbing images for camera2
        camera2.StartGrabbing()

        loops = 0
        counter = 1
        key = 0
        
        #while camera1.IsGrabbing() and camera2.IsGrabbing() and key != 27:
        while True:
            if loops == 0:
                t1_start = perf_counter()

            # Grab and retrieve images from camera1
            grab_result1 = camera1.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
            if grab_result1.GrabSucceeded():
                right_image = grab_result1.Array
                # Process image from camera1 as needed
                # ...

            grab_result1.Release()

            # Grab and retrieve images from camera2
            grab_result2 = camera2.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
            if grab_result2.GrabSucceeded():
                left_image = grab_result2.Array
                # Process image from camera2 as needed
                # ...

            grab_result2.Release()
            
            color_left_image = cv.cvtColor(left_image, cv.COLOR_BAYER_BG2RGB)
            color_right_image = cv.cvtColor(right_image, cv.COLOR_BAYER_BG2RGB)
            
            # rectified_left = cv.remap(color_left_image, map_left_x, map_left_y, cv.INTER_LINEAR)
            # rectified_right = cv.remap(color_right_image, map_right_x, map_right_y, cv.INTER_LINEAR) 
            
            # disparity_map = stereo.compute(rectified_left, rectified_right).astype(np.float32)/16
            # normalized_disparity_map = cv.normalize(disparity_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)

            # depth_map = ((focal_length_pixel * baseline_mm) / (disparity_map+1)).astype(np.float32)
            
            
            # Display the images and disparity map
            # cv.imshow("left colour", color_left_image)
            # cv.imshow("Left",left_image)
            # cv.imshow("Right",right_image)
            # cv.imshow('Left rect', rectified_left)
            # cv.imshow('Disparity Map', normalized_disparity_map)
            # key = cv.waitKey(1)
            
            # cv.imshow("Right",right_image)
            # cv.imshow("Left",left_image)
            # key = cv.waitKey(1)
            
            # if key != -1:
            #     print(key)
            
            # if key == 115:
            #     print("saving")
            #     cv.imwrite(f"left_image_{counter}.png",left_image)
            #     cv.imwrite(f"right_image_{counter}.png",right_image)
            #     counter += 1
            loops += 1
            if loops > 100:
                t1_stop = perf_counter()
                t1_elapsed = ((t1_stop - t1_start) * 1000) / loops # time in ms
                print(f"Time: {t1_elapsed:>5.2f}ms", end="\r")
                loops = 0

            


    finally:
        print("stopping feed")
        # Stop grabbing images for both cameras
        camera1.StopGrabbing()
        camera2.StopGrabbing()
        print("closing cameras")
        # Close both cameras
        camera1.Close()
        camera2.Close()
        print("deleting windows")
        cv.destroyAllWindows()
        print("ended")

if __name__ == "__main__":
    main()
