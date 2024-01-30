import cv2 as cv
import numpy as np
from time import perf_counter
from pypylon import pylon

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

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
    map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right = get_rectifier_maps(0)

    focal_length_pixel = 855
    baseline = 0.03316 # meters
    baseline_mm = baseline*1000 # mm


    pub = rospy.Publisher("/test/Kiwis", String, queue_size=1)
    rospy.init_node("camera_processor", anonymous=True)


    tlf = pylon.TlFactory.GetInstance()

    # Get all available devices
    devices = tlf.EnumerateDevices()

    if len(devices) < 2:
        print("Less than 2 cameras found.")
        return

    # Note: devices are always ordered with the lowest Serial Number (SN) first.
    # Therefore this will always be correct (so long as the cameras arent physically swapped in the holder).
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

        hue_lower=15
        hue_upper=38
        sat_lower=130
        sat_upper=255
        val_lower=20
        val_upper=230
        radiusClose=0
        radiusOpen=1
        minAreaThreshold=350

        key = 0
        while camera_left.IsGrabbing() and camera_right.IsGrabbing() and key != 27 and (not rospy.is_shutdown()):

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

            # x,y,w,h = roi_right
            # cv.rectangle(rectified_right,(x,y),(x+w,y+h),(0,255,0),1)
            # x,y,w,h = roi_left
            # cv.rectangle(rectified_left,(x,y),(x+w,y+h),(0,255,0),1)


            rect_r_cpy = rectified_right.copy()
            rect_l_cpy = rectified_left.copy()
            
            # tdisp_start = perf_counter()
            lowerYellow = (hue_lower, sat_lower, val_lower)
            upperYellow = (hue_upper, sat_upper, val_upper)

            hsv_image = cv.cvtColor(rectified_right, cv.COLOR_BGR2HSV)
            yellow_mask = cv.inRange(hsv_image, lowerYellow, upperYellow)

            kernel_c = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*radiusClose+1, 2*radiusClose+1))
            kernel_o = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*radiusOpen+1, 2*radiusOpen+1))

            closing = cv.morphologyEx(yellow_mask, cv.MORPH_CLOSE, kernel_c)
            opening = cv.morphologyEx(closing, cv.MORPH_OPEN, kernel_o)

            contours, _ = cv.findContours(opening, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if contours is not None:
                for contour in contours:
                    cont_area = cv.contourArea(contour)
                    if cont_area > minAreaThreshold:
                        boundingBox = cv.boundingRect(contour)
                        x,y,w,h = boundingBox
                        rect_r_cpy = cv.rectangle(rect_r_cpy, (x,y), (x+w,y+h), (0,255,0), 2)
                        rect_r_cpy = cv.circle(rect_r_cpy, (x+w//2,y+h//2), 5, (255,0,0), -1)

                        x_start = x+10
                        x_width = w+150
                        if (x_start + x_width >= rectified_left.shape[0]):
                            continue

                        roi_img = rectified_left[x_start:x_start+x_width, y+h//2:y+h//2+1]
                        roi_template = rectified_right[x:x+w, y+h//2:y+h//2+1]

                        match_result = cv.matchTemplate(roi_img, roi_template, cv.TM_SQDIFF_NORMED)
                        _, _, min_loc, _ = cv.minMaxLoc(match_result)

                        rect_l_cpy = cv.circle(rect_l_cpy, (x+w//2+min_loc[0]+10, y+h//2), 5, (255,0,0), -1)

                        disparity = 10 + min_loc[0]
                        depth_mm = focal_length_pixel * baseline_mm / disparity

                        rect_r_cpy = cv.putText(rect_r_cpy, f"{depth_mm}", (x,y), cv.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2)

                        pub.publish(f"x: {x+w//2} | y: {y+h//2} | z: {depth_mm}")

            
            # tdisp_stop = perf_counter()
            
            # tdisp_elapsed = ((tdisp_stop - tdisp_start) * 1000)
            # dispaity_time += tdisp_elapsed

            # Display rectified images side by side
            cv.imshow("Left Rectified", rectified_left)
            cv.imshow("Right Rectified", rectified_right)

            cv.imshow("Left Matched", rect_l_cpy)
            cv.imshow("Kiwi Distances", rect_r_cpy)
            cv.imshow("Kiwi Mask", opening)

            # Display the images and disparity map
            # cv.imshow("Left colour", color_left_image)
            # cv.imshow("Right colour", color_right_image)
            key = cv.waitKey(1)

        print(f"Average time: {total_time/loops:>5.2f}ms")
        # print(f"Average disparity time: {dispaity_time/loops:>5.2f}ms")
    
    except rospy.ROSInterruptException:
        pass

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
    