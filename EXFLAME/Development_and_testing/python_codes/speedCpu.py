import cv2 as cv
import numpy as np
from time import perf_counter
from pypylon import pylon

def get_rectifier_maps(a=0):
    # Load calibration parameters from the text file
    calibration_data = np.loadtxt('stereoParams.txt', delimiter=',')

    # Extract parameters
    intrinsics_left = calibration_data[:3, :3]
    distortion_left = calibration_data[3, :5]
    intrinsics_right = calibration_data[4:7, :3]
    distortion_right = calibration_data[7, :5]
    extrinsics_R = calibration_data[8:11, :3]
    extrinsics_T = calibration_data[11, :3]

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
    map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right = get_rectifier_maps(1)

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


    tlf = pylon.TlFactory.GetInstance()

    # Get all available devices
    devices = tlf.EnumerateDevices()

    if len(devices) < 2:
        print("Less than 2 cameras found.")
        return
    elif len(devices) > 2:
        print("More than 2 cameras found.")
        return

    # Therefore this will always be correct (so long as the cameras arent physically swapped in the holder)
    camera_left = pylon.InstantCamera(tlf.CreateDevice(devices[0]))
    camera_right = pylon.InstantCamera(tlf.CreateDevice(devices[1]))

    camera_left.Open()
    camera_right.Open()

    try:
        # Start grabbing images for camera_left
        camera_left.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        # Start grabbing images for camera2
        camera_right.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        total_time = 0
        loops = 0
        # dispaity_time = 0

        key = 0
        while camera_left.IsGrabbing() and camera_right.IsGrabbing() and key != 27:

            loops += 1
            t1_start = perf_counter()

            grab_result_left = camera_left.RetrieveResult(100)
            if grab_result_left.GrabSucceeded():
                left_image = grab_result_left.Array
            grab_result_left.Release()

            grab_result_right = camera_right.RetrieveResult(100)
            if grab_result_right.GrabSucceeded():
                right_image = grab_result_right.Array
            grab_result_right.Release()

            color_left_image = cv.cvtColor(left_image, cv.COLOR_BAYER_BG2RGB)
            color_right_image = cv.cvtColor(right_image, cv.COLOR_BAYER_BG2RGB)

            rectified_right = cv.remap(color_right_image, map_right_x, map_right_y, cv.INTER_LINEAR)
            rectified_left = cv.remap(color_left_image, map_left_x, map_left_y, cv.INTER_LINEAR)

            t1_stop = perf_counter()
            t1_duration = ((t1_stop - t1_start) * 1000)
            total_time += t1_duration

            x,y,w,h = roi_right
            cv.rectangle(rectified_right,(x,y),(x+w,y+h),(0,255,0),1)
            x,y,w,h = roi_left
            cv.rectangle(rectified_left,(x,y),(x+w,y+h),(0,255,0),1)
            
            # tdisp_start = perf_counter()
            # disparity_map = stereo.compute(rectified_left, rectified_right).astype(np.float32)/16
            # tdisp_stop = perf_counter()
            # normalized_disparity_map = cv.normalize(disparity_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
            # cv.imshow('Disparity Map', normalized_disparity_map)
            # tdisp_elapsed = ((tdisp_stop - tdisp_start) * 1000)
            # dispaity_time += tdisp_elapsed

            # Display rectified images side by side
            cv.imshow("Left Rectified", rectified_left)
            cv.imshow("Right Rectified", rectified_right)

            # Display the images and disparity map
            cv.imshow("Left colour", color_left_image)
            cv.imshow("Right colour", color_right_image)
            key = cv.waitKey(1)

        print(f"Average time: {total_time/loops:>5.2f}ms")
        # print(f"Average disparity time: {dispaity_time/loops:>5.2f}ms")

    finally:
        # Stop grabbing images for both cameras
        camera_left.StopGrabbing()
        camera_right.StopGrabbing()
    
        # Close both cameras
        camera_left.Close()
        camera_right.Close()
    
        cv.destroyAllWindows()
        print("ending")

            

if __name__ == "__main__":
    main()